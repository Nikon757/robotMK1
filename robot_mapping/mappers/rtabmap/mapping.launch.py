#!/usr/bin/env python3
"""RobotMK1 RTAB-Map mapping launch (runs INSIDE the Docker container, ROS 2 Jazzy).

    ros2 launch /robot/mapping.launch.py db_path:=/maps/rtabmap.db viz:=true odom:=vo

Nodes
    depthai_ros_driver camera.launch.py  OAK-D Lite: /oak/rgb/image_rect, /oak/rgb/camera_info,
                                         /oak/stereo/image_raw (depth aligned to RGB), /oak/imu/data
    ldlidar_stl_ros2_node                STL-27L -> /scan_raw (frame "laser")
    scan_fov_filter.py                   /scan_raw -> /scan   (forward 180 deg only)
    static TFs                           base_link -> laser, oak-d_frame -> oak_imu_frame
    imu_filter_madgwick                  /oak/imu/data -> /imu/data (orientation for BMI270 units)
    rtabmap_sync/rgbd_sync               -> /rgbd_image
    rtabmap_odom/rgbd_odometry (or icp_odometry)   odom -> base_link
    rtabmap_slam/rtabmap                 map -> odom, database, /rtabmap/cloud_map, /rtabmap/map
    rtabmap_viz/rtabmap_viz              optional GUI (viz:=true)
    ros2 bag record                      optional raw topics (bag:=true)

Geometry, ports and the LiDAR FOV come from robot.yaml ($ROBOT_CONFIG, default /robot_config/robot.yaml).
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_DIR = os.path.dirname(os.path.abspath(__file__))


def _true(v: str) -> bool:
    return str(v).lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    cfg_path = os.environ.get("ROBOT_CONFIG", "/robot_config/robot.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cam, lid, mp = cfg["camera"], cfg["lidar"], cfg["mapping"]

    db_path = LaunchConfiguration("db_path").perform(context)
    delete_db = _true(LaunchConfiguration("delete_db").perform(context))
    viz = _true(LaunchConfiguration("viz").perform(context))
    odom = LaunchConfiguration("odom").perform(context)
    force_3dof = _true(LaunchConfiguration("force_3dof").perform(context))
    use_imu = _true(LaunchConfiguration("use_imu").perform(context))
    bag = _true(LaunchConfiguration("bag").perform(context))
    bag_dir = LaunchConfiguration("bag_dir").perform(context)
    localization = _true(LaunchConfiguration("localization").perform(context))
    lidar_port = LaunchConfiguration("lidar_port").perform(context)
    qos = 2  # best-effort subscriptions are compatible with reliable AND best-effort publishers

    cm = cam["mount"]
    lm = lid["mount"]
    fwd = math.radians(float(lid["forward_angle_deg"]))
    laser_yaw = math.radians(float(lm.get("yaw", 0.0))) + (fwd if bool(lid.get("clockwise", True)) else -fwd)
    imu_rpy = [math.radians(float(v)) for v in cam.get("imu_rpy_deg", [0.0, 90.0, 0.0])]
    product = {"STL27L": "LDLiDAR_STL27L", "LD19": "LDLiDAR_LD19", "LD06": "LDLiDAR_LD06"}.get(
        str(lid.get("model", "STL27L")).upper(), "LDLiDAR_STL27L")

    actions = []

    # ---------------------------------------------------------------- OAK-D Lite
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(depthai_prefix, "launch", "camera.launch.py")),
        launch_arguments={
            "name": "oak",
            "camera_model": str(cam.get("model", "OAK-D-LITE")),
            "parent_frame": "base_link",
            "cam_pos_x": str(cm["x"]), "cam_pos_y": str(cm["y"]), "cam_pos_z": str(cm["z"]),
            "cam_roll": str(math.radians(float(cm.get("roll", 0.0)))),
            "cam_pitch": str(math.radians(float(cm.get("pitch", 0.0)))),
            "cam_yaw": str(math.radians(float(cm.get("yaw", 0.0)))),
            "params_file": os.path.join(ROBOT_DIR, "params", "oak.yaml"),
            "rectify_rgb": "true",
            "use_rviz": "false",
            "publish_tf_from_calibration": "false",
            "imu_from_descr": "false",
        }.items()))

    # IMU chip orientation inside the camera body (oak-d_frame = camera body frame, x forward)
    actions.append(Node(package="tf2_ros", executable="static_transform_publisher", name="tf_oak_imu",
                        arguments=["--x", "0", "--y", "0", "--z", "0",
                                   "--roll", f"{imu_rpy[0]:.6f}", "--pitch", f"{imu_rpy[1]:.6f}", "--yaw", f"{imu_rpy[2]:.6f}",
                                   "--frame-id", "oak-d_frame", "--child-frame-id", "oak_imu_frame"]))

    # ---------------------------------------------------------------- LiDAR
    actions.append(Node(package="ldlidar_stl_ros2", executable="ldlidar_stl_ros2_node", name="ldlidar", output="screen",
                        parameters=[{"product_name": product, "topic_name": "scan_raw", "frame_id": "laser",
                                     "port_name": lidar_port, "port_baudrate": int(cfg["serial"]["lidar_baud"]),
                                     "laser_scan_dir": bool(lid.get("clockwise", True)),
                                     "enable_angle_crop_func": False, "angle_crop_min": 0.0, "angle_crop_max": 0.0}]))
    actions.append(ExecuteProcess(
        cmd=["python3", os.path.join(ROBOT_DIR, "scan_fov_filter.py"), "--ros-args",
             "-p", "in_topic:=scan_raw", "-p", "out_topic:=scan",
             "-p", f"forward_yaw_rad:={laser_yaw:.6f}", "-p", f"fov_deg:={float(lid['fov_deg'])}",
             "-p", f"range_min:={float(lid['min_range_m'])}", "-p", f"range_max:={float(lid['max_range_m'])}"],
        output="screen"))
    actions.append(Node(package="tf2_ros", executable="static_transform_publisher", name="tf_laser",
                        arguments=["--x", str(lm["x"]), "--y", str(lm["y"]), "--z", str(lm["z"]),
                                   "--roll", str(math.radians(float(lm.get("roll", 0.0)))),
                                   "--pitch", str(math.radians(float(lm.get("pitch", 0.0)))),
                                   "--yaw", f"{laser_yaw:.6f}",
                                   "--frame-id", "base_link", "--child-frame-id", "laser"]))

    # ---------------------------------------------------------------- IMU orientation filter
    actions.append(Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node", name="imu_filter",
                        output="screen",
                        parameters=[{"use_mag": False, "world_frame": "enu", "publish_tf": False, "gain": 0.1,
                                     "zeta": 0.0, "remove_gravity_vector": False, "stateless": False}],
                        remappings=[("imu/data_raw", "/oak/imu/data"), ("imu/data", "/imu/data")]))

    # ---------------------------------------------------------------- RTAB-Map
    rtab_yaml = os.path.join(ROBOT_DIR, "params", "rtabmap.yaml")
    overrides = {
        "Reg/Force3DoF": "true" if force_3dof else "false",
        "Optimizer/GravitySigma": "0.3" if use_imu else "0.0",
        "Mem/IncrementalMemory": "false" if localization else "true",
        "Mem/InitWMWithAllNodes": "true" if localization else "false",
    }
    common = {"frame_id": "base_link", "odom_frame_id": "odom", "map_frame_id": "map", "qos": qos,
              "qos_image": qos, "qos_camera_info": qos, "qos_scan": qos, "qos_imu": qos, "qos_odom": qos,
              "publish_tf": True, "approx_sync": True, "wait_for_transform": 0.3}
    remap_cam = [("rgb/image", "/oak/rgb/image_rect"), ("rgb/camera_info", "/oak/rgb/camera_info"),
                 ("depth/image", "/oak/stereo/image_raw")]

    actions.append(Node(package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync", namespace="rtabmap", output="screen",
                        parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.05, "qos": qos,
                                     "qos_camera_info": qos, "depth_scale": 1.0}],
                        remappings=remap_cam + [("rgbd_image", "/rgbd_image")]))

    odom_remap = [("rgbd_image", "/rgbd_image"), ("scan", "/scan"), ("imu", "/imu/data"), ("odom", "/odom"),
                  ("odom_info", "/odom_info")]
    if odom == "icp":
        actions.append(Node(package="rtabmap_odom", executable="icp_odometry", name="icp_odometry", namespace="rtabmap", output="screen",
                            parameters=[rtab_yaml, dict(common, wait_imu_to_init=use_imu,
                                                        guess_frame_id="", **overrides),
                                        {"Icp/PointToPlane": "false", "Icp/VoxelSize": "0.05",
                                         "Icp/MaxCorrespondenceDistance": "0.3", "Odom/ScanKeyFrameThr": "0.7",
                                         "OdomF2M/ScanSubtractRadius": "0.05", "OdomF2M/ScanMaxSize": "15000"}],
                            remappings=odom_remap))
    else:
        actions.append(Node(package="rtabmap_odom", executable="rgbd_odometry", name="rgbd_odometry", namespace="rtabmap", output="screen",
                            parameters=[rtab_yaml, dict(common, subscribe_rgbd=True, approx_sync=False,
                                                        wait_imu_to_init=use_imu, guess_frame_id="", **overrides)],
                            remappings=odom_remap))

    rtab_params = dict(common, subscribe_rgbd=True, subscribe_scan=True, subscribe_depth=False, subscribe_rgb=False,
                       subscribe_odom_info=True, database_path=db_path, approx_sync=True,
                       approx_sync_max_interval=0.2, **overrides)
    actions.append(Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap", output="screen",
                        parameters=[rtab_yaml, rtab_params],
                        remappings=odom_remap,
                        arguments=["-d"] if (delete_db and not localization) else []))

    if viz:
        actions.append(Node(package="rtabmap_viz", executable="rtabmap_viz", name="rtabmap_viz", namespace="rtabmap", output="screen",
                            parameters=[dict(common, subscribe_rgbd=True, subscribe_scan=True,
                                             subscribe_odom_info=True, approx_sync=True, approx_sync_max_interval=0.2)],
                            remappings=odom_remap))

    if bag:
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "mcap", "-o", bag_dir,
                 "/oak/rgb/image_rect", "/oak/rgb/camera_info", "/oak/stereo/image_raw", "/oak/imu/data",
                 "/scan_raw", "/scan", "/tf", "/tf_static", "/odom"],
            output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("db_path", default_value="/maps/rtabmap.db"),
        DeclareLaunchArgument("delete_db", default_value="true", description="start a fresh database"),
        DeclareLaunchArgument("viz", default_value="false", description="run rtabmap_viz (needs X11)"),
        DeclareLaunchArgument("odom", default_value="vo", description="vo (rgbd_odometry) | icp (icp_odometry, LiDAR)"),
        DeclareLaunchArgument("force_3dof", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="false", description="gravity constraints; needs calib-imu"),
        DeclareLaunchArgument("bag", default_value="false"),
        DeclareLaunchArgument("bag_dir", default_value="/maps/bag"),
        DeclareLaunchArgument("localization", default_value="false", description="reuse db, do not add nodes"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/robot_lidar"),
        OpaqueFunction(function=launch_setup),
    ])
