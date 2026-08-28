from setuptools import find_packages, setup


package_name = "cuvslam_sim_sensor_adapter"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="tseng",
    maintainer_email="tseng@example.com",
    description="Simulation-only sensor normalization for cuVSLAM.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "right_camera_info_adapter = "
                "cuvslam_sim_sensor_adapter.right_camera_info_adapter:main"
            ),
        ],
    },
)
