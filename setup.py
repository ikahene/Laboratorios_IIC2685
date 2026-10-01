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
        'launch/teleop.xml',
        'launch/avanzar_y_rotar.xml',
        ]),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscarmnz',
    maintainer_email='oscar.merino295@gmail.com',
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
            'teleop = lab1.teleop:main',
            'pose_loader = lab1.pose_loader:main',
        ],
    },
)
