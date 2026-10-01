import os
from glob import glob
from setuptools import setup

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime Gibson',
    maintainer_email='jgibsonu@estudiante.uc.cl',
    description='Laboratorio 1 - Operaciones básicas de movimiento y percepción',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Formato: 'nombre_ejecutable = carpeta.archivo:main'
            'dead_reckoning_nav = lab1.move_turtlebot:main',
            'pose_loader = lab1.pose_loader:main',
            'obstacle_detector = lab1.obstacle_detector:main',
        ],
    },
)