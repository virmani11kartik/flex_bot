from setuptools import setup
import os
from glob import glob

package_name = "flex_bot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.lua")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kartik",
    maintainer_email="kartik8virmani@gmail.com",
    description="flex_bot bringup: sick picoscan + static TF + EKF (+ optional RViz) + SLAM launcher",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
