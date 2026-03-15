from setuptools import find_packages
from setuptools import setup


package_name = "gi5651_can_odom"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            ["config/can_2_txt.yaml", "config/txt_2can.yaml"],
        ),
        (
            f"share/{package_name}/launch",
            ["launch/gi5651_can_odom.launch.py", "launch/txt_2can.launch.py"],
        ),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lzz",
    maintainer_email="lzz@example.com",
    description="Read GI5651 CAN frames from SocketCAN and publish ROS2 odometry.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gi5651_can_odom_node = gi5651_can_odom.gi5651_can_odom_node:main",
            "gi5651_txt_2_can_node = gi5651_can_odom.gi5651_txt_2_can_node:main",
        ],
    },
)
