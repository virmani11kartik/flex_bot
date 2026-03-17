from setuptools import setup

package_name = "sorting_trigger_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="bemi@example.com",
    description="HTTP -> ROS2 trigger bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge = sorting_trigger_bridge.bridge:main",
        ],
    },
)
