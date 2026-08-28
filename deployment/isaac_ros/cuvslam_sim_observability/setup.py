from glob import glob
import os

from setuptools import find_packages, setup


package_name = "cuvslam_sim_observability"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.sdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tseng",
    maintainer_email="tseng@example.com",
    description=(
        "Simulation-only ground-truth visualization for cuVSLAM evaluation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "world_outline_publisher = "
                "cuvslam_sim_observability.world_outline_publisher:main"
            ),
        ],
    },
)
