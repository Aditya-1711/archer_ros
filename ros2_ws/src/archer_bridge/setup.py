from setuptools import find_packages, setup

package_name = "archer_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Required by ament — makes the package discoverable by ros2 run
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Boss Aditya",
    maintainer_email="archer@ros.local",
    description="Archer AI-to-ROS2 bridge node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run archer_bridge bridge_node
            "bridge_node = archer_bridge.bridge_node:main",
            "depth_to_pointcloud = archer_bridge.depth_to_pointcloud:main",
        ],
    },
)
