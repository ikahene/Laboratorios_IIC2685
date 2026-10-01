from setuptools import find_packages, setup

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/avanzar_y_rotar.xml',
            'launch/teleop.xml',
            'launch/accion_y_percepcion.xml',
        ]),
        ('share/' + package_name + '/config', [
            'lab1/poses.txt',
            'lab1/cuadrado.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikahene',
    maintainer_email='ikahene@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dead_reckoning_nav = lab1.move_turtlebot:main',
            'pose_loader = lab1.pose_loader:main',
            'teleop = lab1.teleop:main',
            'obstacle_detector = lab1.obstacle_detector:main',
        ],
    },
)
