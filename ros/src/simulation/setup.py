import os
from glob import glob

from setuptools import find_packages, setup

package_name = "simulation"

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
            os.path.join("share", package_name, "description"),
            glob("description/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sebtheiler,jaxonpoentis",
    maintainer_email="25444757+sebtheiler@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "random_control = simulation.demo_control:main",
        ],
    },
)
