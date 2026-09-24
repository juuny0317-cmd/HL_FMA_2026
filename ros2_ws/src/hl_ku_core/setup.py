from glob import glob
from setuptools import find_packages, setup


package_name = "hl_ku_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        (
            "share/" + package_name + "/config/mando",
            glob("config/mando/*.yaml"),
        ),
        (
            "share/" + package_name + "/config/mando/competition",
            glob("config/mando/competition/*.yaml"),
        ),
        ("share/" + package_name + "/routes", glob("routes/*.csv")),
        (
            "share/" + package_name + "/routes/mando",
            glob("routes/mando/*.csv"),
        ),
        (
            "share/" + package_name + "/routes/mando/competition",
            glob("routes/mando/competition/*.csv"),
        ),
        (
            "share/" + package_name + "/routes/course_07_parking_overlay",
            glob("routes/course_07_parking_overlay/*"),
        ),
        (
            "share/" + package_name + "/weights/integrated_7class",
            glob("weights/integrated_7class/*.pt"),
        ),
        (
            "share/" + package_name + "/weights/endpoint_2class_v3",
            glob("weights/endpoint_2class_v3/*.pt"),
        ),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="HL KU Team",
    maintainer_email="hl-ku@example.com",
    description="GNSS-first autonomous driving foundation for HL FMA 1/5.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "um982_serial = hl_ku_core.um982_serial_node:main",
            "ntrip_client = hl_ku_core.ntrip_client_node:main",
            "gnss_localizer = hl_ku_core.gnss_localizer_node:main",
            "velocity_selector = hl_ku_core.velocity_selector_node:main",
            "camera_perception = hl_ku_core.camera_perception_node:main",
            "yolo_perception = hl_ku_core.yolo_perception_node:main",
            "endpoint_perception = hl_ku_core.endpoint_perception_node:main",
            "lidar_perception = hl_ku_core.lidar_perception_node:main",
            "rplidar_a3 = hl_ku_core.rplidar_a3_node:main",
            "scan_obstacle_mapper = hl_ku_core.scan_obstacle_mapper_node:main",
            "perception_aggregator = hl_ku_core.perception_aggregator_node:main",
            "path_tracker = hl_ku_core.path_tracker_node:main",
            "mission_manager = hl_ku_core.mission_manager_node:main",
            "vehicle_controller = hl_ku_core.vehicle_controller_node:main",
            "safety_supervisor = hl_ku_core.safety_supervisor_node:main",
            "mcu_udp_bridge = hl_ku_core.mcu_udp_bridge_node:main",
            "nucleo_serial_bridge = hl_ku_core.nucleo_serial_bridge_node:main",
            "nucleo_serial_mux = hl_ku_core.nucleo_mux_host:main",
            "gnss_survey = hl_ku_core.gnss_survey_node:main",
            "route_recorder = hl_ku_core.route_recorder_node:main",
            "route_prepare = hl_ku_core.route_prepare:main",
            "rtk_waypoint_recorder = hl_ku_core.rtk_waypoint_recorder_node:main",
            "keyboard_teleop = hl_ku_core.keyboard_teleop_node:main",
            "preflight_check = hl_ku_core.preflight_check_node:main",
            "route_drive_session = hl_ku_core.route_drive_session:main",
            "mando_tui = hl_ku_core.mando_tui:main",
            "mando_route_slice = hl_ku_core.mando_route_slice:main",
            "mission_tagging = hl_ku_core.mission_tagging:main",
            "local_detour = hl_ku_core.local_detour_node:main",
        ]
    },
)
