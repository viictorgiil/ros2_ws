from setuptools import find_packages, setup

package_name = 'tictactoe_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['hand_landmarker.task'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victor.gil.ferrer@gmail.com',
    description='Vision package for tic-tac-toe robot',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = tictactoe_vision.vision:main',
            'bridge_node = tictactoe_vision.bridge_node:main',
        ],
    },
)