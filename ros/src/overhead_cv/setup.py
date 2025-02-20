import os
from glob import glob

from setuptools import find_packages, setup

package_name = "overhead_cv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="observer",
    maintainer_email="jaximus808@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_feed=overhead_cv.camera_feed:main",
            "robot_tracker=overhead_cv.robot_tracker:main",
            "position_estimator=overhead_cv.position_estimator:main",
        ],
    },
)
