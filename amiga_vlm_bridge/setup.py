from setuptools import find_packages, setup


package_name = "amiga_vlm_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "requests"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="ROS 2 node: VLM loop on latest camera frame, append responses to one file.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vlm_server = amiga_vlm_bridge.vlm_server:main",
        ],
    },
)

