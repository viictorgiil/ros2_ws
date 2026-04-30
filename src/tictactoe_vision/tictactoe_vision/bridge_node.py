import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading

HOST = 'localhost'
PORT = 65432

class BridgeNode(Node):
    def __init__(self):
        super().__init__('vision_bridge')
        self.publisher = self.create_publisher(String, '/tictactoe/tablero', 10)
        self.get_logger().info('Bridge iniciado, esperando conexión del nodo visión...')
        
        # Servidor socket en hilo separado
        self.thread = threading.Thread(target=self.servidor_socket)
        self.thread.daemon = True
        self.thread.start()

    def servidor_socket(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(1)
            while True:
                conn, addr = s.accept()
                self.get_logger().info(f'Nodo visión conectado: {addr}')
                with conn:
                    buffer = ""
                    while True:
                        data = conn.recv(1024).decode()
                        if not data:
                            break
                        buffer += data
                        while "\n" in buffer:
                            linea, buffer = buffer.split("\n", 1)
                            try:
                                datos = json.loads(linea)
                                msg = String()
                                msg.data = json.dumps(datos)
                                self.publisher.publish(msg)
                            except json.JSONDecodeError:
                                pass

def main():
    rclpy.init()
    node = BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
