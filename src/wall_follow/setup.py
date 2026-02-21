from setuptools import find_packages, setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bemi',
    maintainer_email='gbemigao@seas.upenn.edu',
    description='Wall following using LaserScan and wheel rad/s commands',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'wall_follow = wall_follow.wall_follow:main',
            'lidar_wallfollow_debug = wall_follow.lidar_wallfollow_debug:main',
        ],
    },
)
