from glob import glob
import os

from setuptools import setup

package_name = "cleaning_robot_coverage"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aderfd",
    maintainer_email="aderfd106@gmail.com",
    description="Boustrophedon coverage path planner for occupancy grid maps.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coverage_planner = cleaning_robot_coverage.coverage_planner:main",
            "coverage_progress_publisher = cleaning_robot_coverage.coverage_progress_publisher:main",
            "cleaning_task_manager = cleaning_robot_coverage.cleaning_task_manager:main",
        ],
    },
)
