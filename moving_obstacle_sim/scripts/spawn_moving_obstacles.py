#!/usr/bin/env python3

import os
import sys
import math

import rospy
import rospkg
import yaml
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from model_builders import build_model_sdf


def _to_float_list(value, length, default):
    if not isinstance(value, (list, tuple)):
        return list(default)
    out = list(default)
    for i in range(min(length, len(value))):
        try:
            out[i] = float(value[i])
        except (TypeError, ValueError):
            out[i] = default[i]
    return out


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def resolve_config_path(config_ref):
    if not isinstance(config_ref, str) or not config_ref:
        raise ValueError("Obstacle config path is empty.")

    if config_ref.startswith("pkg://"):
        payload = config_ref[len("pkg://") :]
        split_idx = payload.find("/")
        if split_idx <= 0:
            raise ValueError("Invalid pkg:// config format. Expected pkg://<pkg>/<path>.")
        pkg_name = payload[:split_idx]
        rel_path = payload[split_idx + 1 :]
        pkg_root = rospkg.RosPack().get_path(pkg_name)
        return os.path.join(pkg_root, rel_path)

    if os.path.isabs(config_ref):
        return config_ref

    raise ValueError("Only pkg:// and absolute paths are supported. Got: {}".format(config_ref))


def load_config(config_ref):
    config_path = resolve_config_path(config_ref)
    if not os.path.exists(config_path):
        raise FileNotFoundError("Obstacle config does not exist: {}".format(config_path))

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    obstacles = data.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise ValueError("`obstacles` must be a list in {}".format(config_path))

    return config_path, obstacles


def obstacle_pose_from_cfg(obstacle):
    pose_cfg = obstacle.get("pose", {}) if isinstance(obstacle.get("pose", {}), dict) else {}
    xyz = _to_float_list(pose_cfg.get("xyz", [0.0, 0.0, 0.0]), 3, [0.0, 0.0, 0.0])
    rpy = _to_float_list(pose_cfg.get("rpy", [0.0, 0.0, 0.0]), 3, [0.0, 0.0, 0.0])

    pose = Pose()
    pose.position = Point(*xyz)
    pose.orientation = rpy_to_quaternion(rpy[0], rpy[1], rpy[2])
    return pose


def spawn_obstacles(obstacles, reference_frame="world", delete_if_exists=True):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    delete_proxy = None
    if delete_if_exists:
        rospy.wait_for_service("/gazebo/delete_model")
        delete_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    spawned = 0
    used_names = set()

    for idx, obstacle in enumerate(obstacles):
        if not isinstance(obstacle, dict):
            rospy.logwarn("[spawn_moving_obstacles] Skip non-dict obstacle at index %d", idx)
            continue

        if not bool(obstacle.get("enabled", True)):
            rospy.loginfo("[spawn_moving_obstacles] Skip disabled obstacle at index %d", idx)
            continue

        base_name = str(obstacle.get("name", "moving_obstacle_{}".format(idx))).strip() or "moving_obstacle_{}".format(idx)
        model_name = base_name
        suffix = 1
        while model_name in used_names:
            model_name = "{}_{}".format(base_name, suffix)
            suffix += 1

        used_names.add(model_name)
        obstacle["name"] = model_name

        pose = obstacle_pose_from_cfg(obstacle)
        sdf_xml = build_model_sdf(obstacle, obstacle_index=idx)

        if delete_proxy is not None:
            try:
                delete_proxy(model_name)
            except rospy.ServiceException:
                pass

        try:
            resp = spawn_proxy(model_name, sdf_xml, "", pose, reference_frame)
        except rospy.ServiceException as exc:
            rospy.logerr("[spawn_moving_obstacles] Spawn service failed for %s: %s", model_name, str(exc))
            continue

        if not resp.success:
            rospy.logerr("[spawn_moving_obstacles] Failed to spawn %s: %s", model_name, resp.status_message)
            continue

        rospy.loginfo("[spawn_moving_obstacles] Spawned %s", model_name)
        spawned += 1

    return spawned


def main():
    rospy.init_node("spawn_moving_obstacles")

    config_ref = rospy.get_param("~config", "pkg://moving_obstacle_sim/config/moving_obstacles_demo.yaml")
    reference_frame = rospy.get_param("~reference_frame", "world")
    delete_if_exists = bool(rospy.get_param("~delete_if_exists", False))

    try:
        resolved_path, obstacles = load_config(config_ref)
    except Exception as exc:
        rospy.logerr("[spawn_moving_obstacles] %s", str(exc))
        raise

    rospy.loginfo("[spawn_moving_obstacles] Using config: %s", resolved_path)
    count = spawn_obstacles(obstacles, reference_frame=reference_frame, delete_if_exists=delete_if_exists)
    rospy.loginfo("[spawn_moving_obstacles] Done. Spawned %d obstacles.", count)


if __name__ == "__main__":
    main()
