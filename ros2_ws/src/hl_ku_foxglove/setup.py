from glob import glob

from setuptools import find_packages, setup


package_name = "hl_ku_foxglove"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="HL KU Team",
    maintainer_email="hl-ku@example.com",
    description="Safe rosbag replay and Foxglove visualization for HL KU FMA.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "replay_visualizer = hl_ku_foxglove.replay_visualizer_node:main",
            "live_status_visualizer = hl_ku_foxglove.live_status_visualizer_node:main",
        ]
    },
)
