from pathlib import Path
import cv2
import socket
import struct
import json
import time
import numpy as np
import mediapipe as mp

# ── Connections ─────────────────────────────────────────────────────────────────
#CAM_HOST    = '0.0.0.0'
#CAM_PORT    = 5001
BRIDGE_HOST = 'localhost'
BRIDGE_PORT = 65432

# ── Classification parameters ───────────────────────────────────────────────────
EMPTY_THRESH = 0.10

# ── Rectified dimensions (horizontal: storage1 | board | storage2) ──────────────
PROP_STORAGE1 = 10 / 40
PROP_BOARD  = 20 / 40
PROP_STORAGE2 = 10 / 40

RECT_W = 600
RECT_H = 300

MARGEN_RATIO = 0.10

DST_CORNERS = np.array([
    [0,      0     ],
    [RECT_W, 0     ],
    [RECT_W, RECT_H],
    [0,      RECT_H]
], dtype=np.float32)

homography  = None
M_rotation  = None
detection_failures = 0
MAX_FAILURES = 3


# ── Green marker detection ──────────────────────────────────────────────────────

def detect_green_markers(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (62, 93, 77), (87, 255, 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centroids = []
    for c in contours:
        if cv2.contourArea(c) > 50:
            M = cv2.moments(c)
            if M['m00'] > 0:
                centroids.append((M['m10'] / M['m00'], M['m01'] / M['m00']))

    print(f"Green markers detected: {len(centroids)}")
    cv2.imwrite('/tmp/mask_verde.png', mask)

    if len(centroids) != 5:
        return None, None

    pts = np.array(centroids, dtype=np.float32)

    # Central point = minimum maximum distance to the rest
    max_dists = [max(np.linalg.norm(pts[i] - pts[j]) for j in range(5) if j != i) for i in range(5)]
    idx_center   = int(np.argmin(max_dists))
    center       = pts[idx_center]
    corners_pts = np.delete(pts, idx_center, axis=0)

    suma = corners_pts.sum(axis=1)
    dif  = np.diff(corners_pts, axis=1).flatten()
    tl = corners_pts[np.argmin(suma)]
    br = corners_pts[np.argmax(suma)]
    tr = corners_pts[np.argmin(dif)]
    bl = corners_pts[np.argmax(dif)]

    return np.array([tl, tr, br, bl], dtype=np.float32), center


# ── Homography ──────────────────────────────────────────────────────────────────

def calcular_homography(frame):
    """
    1. Detect markers in original frame → calculate angle → M_rotation
    2. Rotate frame with that angle → frame_rotated
    3. Detect markers in frame_rotated → calculate H directly on it
    """
    global homography, M_rotation

    global detection_failures
    # Step 1: detect markers in original frame to get angle
    corners, center = detect_green_markers(frame)
    if corners is None or center is None:
        detection_failures += 1
        return None, None
    detection_failures = 0  # reset si detecta bien

    tl, tr, br, bl = corners[0], corners[1], corners[2], corners[3]

    def dist_segmento(p, a, b):
        ab = b - a
        t  = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-8), 0, 1)
        return np.linalg.norm((p - a) - t * ab)

    edges = [('TL-TR', tl, tr), ('TR-BR', tr, br), ('BR-BL', br, bl), ('BL-TL', bl, tl)]
    edge_name, pa, pb = min(edges, key=lambda x: dist_segmento(center, x[1], x[2]))
    angle_deg = np.degrees(np.arctan2(pb[1] - pa[1], pb[0] - pa[0]))
    print(f"  Edge: {edge_name}, angle={angle_deg:.1f}")

    # Step 2: rotate frame
    h, w = frame.shape[:2]
    M_rot = cv2.getRotationMatrix2D((w / 2, h / 2), angle_deg, 1.0)
    frame_rot = cv2.warpAffine(frame, M_rot, (w, h))
    M_rotation = M_rot

    # Step 3: detect markers in rotated frame and calculate H
    corners_rot, _ = detect_green_markers(frame_rot)
    if corners_rot is not None:
        H, _ = cv2.findHomography(corners_rot, DST_CORNERS)
        if H is not None:
            homography = H
            print("  Homography calculated on rotated frame OK")

    return corners, center


def rectify(frame_rotated):
    if homography is None:
        return None
    return cv2.warpPerspective(frame_rotated, homography, (RECT_W, RECT_H))


# ── Zone geometry ───────────────────────────────────────────────────────────────

def get_zonas():
    x1_s1 = int(RECT_W * PROP_STORAGE1)
    x1_board = int(RECT_W * (PROP_STORAGE1 + PROP_BOARD))
    return {
        'storage1': (0,     x1_s1),
        'board':  (x1_s1, x1_board),
        'storage2': (x1_board, RECT_W),
    }


def get_cells_zona(x0, x1, cols_config, col_margin_extra=None, col_square=None, col_offset=None):
    """
    col_margin_extra: dict {col_idx: extra_factor} to reduce margin in specific columns
    col_square: set of col_idx where cells should be square
    """
    n_cols     = len(cols_config)
    zone_width = x1 - x0
    col_width  = zone_width / n_cols
    cells   = []
    for col_idx, n_pieces in enumerate(cols_config):
        offset = col_offset.get(col_idx, 0) if col_offset else 0
        cx0  = x0 + int(col_idx * col_width) + offset
        cx1_ = x0 + int((col_idx + 1) * col_width) + offset
        cell_height = RECT_H / n_pieces

        # Extra margin for specified columns
        extra = col_margin_extra.get(col_idx, 0) if col_margin_extra else 0
        margin_x = int(col_width  * (MARGEN_RATIO - extra))
        margin_y = int(cell_height * (MARGEN_RATIO - extra))

        for fila in range(n_pieces):
            ry1 = int(fila * cell_height) + margin_y
            ry2 = int((fila + 1) * cell_height) - margin_y

            x1_cas = cx0 + margin_x
            x2_cas = cx1_ - margin_x

            # Casilla cuadrada: centrada en su celda
            if col_square and col_idx in col_square:
                cx_mid = (cx0 + cx1_) // 2
                # Compress vertically toward storage center
                # usando 1/3 y 2/3 del alto total en lugar de 1/4 y 3/4
                center_storage = RECT_H / 2
                offset = int(cell_height * 0.25)  # acercar al center
                if fila == 0:
                    cy_mid = int(cell_height / 2) + offset
                else:
                    cy_mid = int(RECT_H - cell_height / 2) - offset
                lado   = min(int(col_width * (1 - MARGEN_RATIO)),
                             int(cell_height * (1 - MARGEN_RATIO)))
                x1_cas = cx_mid - lado // 2
                x2_cas = cx_mid + lado // 2
                ry1    = cy_mid - lado // 2
                ry2    = cy_mid + lado // 2

            cells.append((x1_cas, ry1, x2_cas, ry2))
    return cells


def get_todas_cells():
    zonas = get_zonas()
    x0_s1, x1_s1 = zonas['storage1']
    x0_board, x1_board = zonas['board']
    x0_s2, x1_s2 = zonas['storage2']

    d3 = int((x1_s1 - x0_s1) * 0.10)
    d2 = int((x1_s1 - x0_s1) * 0.03)

    # Storage1: [2, 3]
    s1_raw = get_cells_zona(
        x0_s1, x1_s1, [2, 3],
        col_margin_extra={1: 0.02},
        col_square={0},
        col_offset={0: -d2, 1: -d3}
    )

    # Orden original s1_raw:
    # col0 top, col0 bottom, col1 top, col1 mid, col1 bottom
    cells_s1 = [
        s1_raw[4],  # col cerca board abajo
        s1_raw[1],  # col lejos board abajo
        s1_raw[3],  # col cerca board medio
        s1_raw[0],  # col cerca board arriba
        s1_raw[2],  # col lejos board arriba
    ]

    # Board: izquierda a derecha, arriba a abajo
    board_raw = get_cells_zona(x0_board, x1_board, [3, 3, 3])

    # Orden original: col0 top/mid/bottom, col1 top/mid/bottom, col2 top/mid/bottom
    cells_board = [
        board_raw[0], board_raw[3], board_raw[6],  # fila arriba
        board_raw[1], board_raw[4], board_raw[7],  # fila medio
        board_raw[2], board_raw[5], board_raw[8],  # fila abajo
    ]

    # Storage2: [3, 2]
    s2_raw = get_cells_zona(
        x0_s2, x1_s2, [3, 2],
        col_margin_extra={0: 0.02},
        col_square={1},
        col_offset={0: d3, 1: d2}
    )

    # Orden original s2_raw:
    # col0 top, col0 mid, col0 bottom, col1 top, col1 bottom
    cells_s2 = [
        s2_raw[2],  # col cerca board abajo
        s2_raw[4],  # col lejos board abajo
        s2_raw[1],  # col cerca board medio
        s2_raw[3],  # col cerca board arriba
        s2_raw[0],  # col lejos board arriba
    ]

    return cells_s1, cells_board, cells_s2



# ── HSV Classifier ──────────────────────────────────────────────────────────────

def classify_cell_roi(roi):
    if roi is None or roi.size == 0:
        return 'EMPTY', 0.0
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_rojo = cv2.bitwise_or(
        cv2.inRange(hsv, (0,   80, 50), (10,  255, 255)),
        cv2.inRange(hsv, (160, 80, 50), (180, 255, 255))
    )
    mask_azul = cv2.inRange(hsv, (90, 60, 60), (130, 255, 255))
    total = roi.shape[0] * roi.shape[1]
    if total == 0:
        return 'EMPTY', 0.0
    ratio_rojo = cv2.countNonZero(mask_rojo) / total
    ratio_azul = cv2.countNonZero(mask_azul) / total
    if ratio_rojo < EMPTY_THRESH and ratio_azul < EMPTY_THRESH:
        return 'EMPTY', 0.0
    return ('O', ratio_rojo) if ratio_rojo > ratio_azul else ('X', ratio_azul)


def classify_zone(rect, cells):
    return [classify_cell_roi(rect[y1:y2, x1:x2])[0] for (x1, y1, x2, y2) in cells]


# ── Visual debug ────────────────────────────────────────────────────────────────

def draw_zone(rect, cells, results, border_color):
    for (x1, y1, x2, y2), label in zip(cells, results):
        cv2.rectangle(rect, (x1, y1), (x2, y2), border_color, 1)
        color_txt = (0, 200, 0) if label == 'EMPTY' else (220, 0, 0) if label == 'O' else (0, 0, 220)
        cv2.putText(rect, label, ((x1+x2)//2 - 25, (y1+y2)//2 + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_txt, 1)
    return rect


def draw_zone_lines(rect):
    for nombre, (x0, x1) in get_zonas().items():
        cv2.line(rect, (x0, 0), (x0, RECT_H), (200, 200, 200), 1)
        cv2.line(rect, (x1, 0), (x1, RECT_H), (200, 200, 200), 1)
        cv2.putText(rect, nombre.upper(), (x0 + 5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    return rect


def draw_board_not_detected(frame):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, "Board not detected.",
                (w//2 - 200, h//2 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 100, 255), 3)
    cv2.putText(frame, "Please adjust the camera.",
                (w//2 - 230, h//2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 100, 255), 2)
    return frame


def draw_hand_warning(frame):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 50), (0, 100, 200), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, "HAND DETECTED - waiting...",
                (15, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return frame


# ── MediaPipe ───────────────────────────────────────────────────────────────────

base_options = mp.tasks.BaseOptions(model_asset_path=str(Path(__file__).with_name('hand_landmarker.task')))
options = mp.tasks.vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=mp.tasks.vision.RunningMode.VIDEO,
    num_hands=2,
    min_hand_detection_confidence=0.5,
    min_hand_presence_confidence=0.5,
    min_tracking_confidence=0.5
)
detector  = mp.tasks.vision.HandLandmarker.create_from_options(options)
_mp_ts    = 0


def draw_hands(frame):
    global _mp_ts
    h, w = frame.shape[:2]
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                        data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    _mp_ts += 1
    result = detector.detect_for_video(mp_image, _mp_ts)
    hand   = len(result.hand_landmarks) > 0
    if hand:
        for hand in result.hand_landmarks:
            for lm in hand:
                cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 4, (0, 165, 255), -1)
    return frame, hand


# ── Frame reception ─────────────────────────────────────────────────────────────

def recibir_frame(sock):
    raw = sock.recv(4)
    if not raw:
        return None
    size = struct.unpack('>I', raw)[0]
    data = b''
    while len(data) < size:
        pkt = sock.recv(size - len(data))
        if not pkt:
            return None
        data += pkt
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


# ── Main ────────────────────────────────────────────────────────────────────────

def main():
    # print("Waiting for camera on port 5001...")
    # cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # cam_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # cam_server.bind((CAM_HOST, CAM_PORT))
    # cam_server.listen(1)
    # cam_sock, addr = cam_server.accept()
    # print(f"Camera connected: {addr}")
    print("Opening camera...")

    # The path /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 will continue to point to the correct C920:
    CAMERA_DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Error: camera not opened")
        return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 20)
    cap.set(cv2.CAP_PROP_ZOOM, 100)

    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
    cap.set(cv2.CAP_PROP_CONTRAST, 140)
    cap.set(cv2.CAP_PROP_SATURATION, 160)
    cap.set(cv2.CAP_PROP_SHARPNESS, 180)
    cap.set(cv2.CAP_PROP_GAIN, 0)

    for _ in range(15):
        cap.read()

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def normalizar_brillo(frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l_eq = clahe.apply(l)
        lab_eq = cv2.merge((l_eq, a, b))
        return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

    bridge_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    bridge_sock.connect((BRIDGE_HOST, BRIDGE_PORT))
    print("Bridge OK")

    last_state = None
    cells_s1, cells_board, cells_s2 = get_todas_cells()
    RECALC_CADA = 30
    frame_count  = 0

    try:
        while True:
            #frame = recibir_frame(cam_sock)
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            frame = normalizar_brillo(frame)

            frame_count += 1
            frame, hand = draw_hands(frame)

            if hand:
                cv2.imshow("Original View", draw_hand_warning(frame))
                cv2.waitKey(1)
                continue

            # Recalculate homography every N frames
            if frame_count % RECALC_CADA == 1:
                calcular_homography(frame)

            cv2.imshow("Original View", frame)

            # Show rotated frame
            if M_rotation is not None:
                h_f, w_f = frame.shape[:2]
                frame_rotated = cv2.warpAffine(frame, M_rotation, (w_f, h_f))
                cv2.imshow("Rotated Frame", frame_rotated)
            else:
                frame_rotated = frame
                cv2.putText(frame, "Searching for green markers...",
                            (15, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            # Check if board is not detected
            if detection_failures >= MAX_FAILURES:
                draw_board_not_detected(frame)
                cv2.imshow("Original View", frame)
                cv2.waitKey(1)
                continue

            # Rectify on rotated frame
            rect = rectify(frame_rotated)

            if rect is not None:
                res_s1 = classify_zone(rect, cells_s1)
                res_board = classify_zone(rect, cells_board)
                res_s2 = classify_zone(rect, cells_s2)

                estado = {"board": res_board, "storage1": res_s1,
                          "storage2": res_s2, "timestamp": time.time()}
                estado_cmp = {k: v for k, v in estado.items() if k != "timestamp"}
                if estado_cmp != last_state:
                    print(f"Board:    {res_board}")
                    print(f"Storage1: {res_s1}")
                    print(f"Storage2: {res_s2}")
                    last_state = estado_cmp
                    bridge_sock.sendall((json.dumps(estado) + "\n").encode())

                debug = rect.copy()
                draw_zone_lines(debug)
                draw_zone(debug, cells_s1, res_s1, (0,   200, 255))
                draw_zone(debug, cells_board, res_board, (255, 255, 255))
                draw_zone(debug, cells_s2, res_s2, (0,   200, 255))
                cv2.imshow("Rectified View", debug)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        detector.close()
        # cam_sock.close()
        # cam_server.close()
        cap.release()
        bridge_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
