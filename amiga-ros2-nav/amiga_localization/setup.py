import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wheel_odometry_node = amiga_localization.wheel_odom:main",
            "bno085_node = amiga_localization.bno085_node:main",
            "imu_orientation_plot = amiga_localization.imu_orientation_plot:main",
        ],
    },
)
