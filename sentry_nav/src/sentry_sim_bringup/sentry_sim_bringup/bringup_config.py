# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Topic, frame and parameter configuration for the dual sentry/manual scene."""

from copy import deepcopy
import math
import os
import re
import xml.etree.ElementTree as ET


# Face the north tunnel along its centerline, within 1 m of the original spawn.
DEFAULT_POSE = ('-4.6', '3.475', '0.3', '0.0')
# Independent map-frame base_footprint guess; never derived from the simulator spawn.
DEFAULT_INITIAL_POSE = ('-4.6', '3.475', '0.0', '0.0')
DEFAULT_PLANNER = 'tdt_yastar_planner/YastarPlanner'
DEFAULT_SPIN_SPEED = '0.0'
DEFAULT_SPAWN_DELAY = '25.0'
FRAME_KEYS = {
    'global_frame', 'local_frame', 'robot_base_frame', 'fake_robot_base_frame',
    'odom_frame', 'base_frame', 'lidar_frame', 'target_frame', 'sensor_frame',
    'control_frame',
}
ROBOT_FRAMES = {
    'odom', 'chassis', 'base_footprint', 'gimbal_yaw_fake', 'gimbal_yaw',
    'gimbal_yaw_odom', 'lidar_link', 'front_mid360',
}
TOPIC_KEYS = {
    'topic', 'odom_topic', 'local_plan_topic', 'input_cmd_vel_topic',
    'output_cmd_vel_topic', 'cmd_spin_topic', 'lid_topic', 'imu_topic',
    'state_estimation_topic', 'registered_scan_topic', 'input_cloud_topic',
}
BT_KEYS = {'default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'}


def normalized_namespace(namespace):
    """Accept root or a valid ROS namespace without leading/trailing slashes."""
    result = namespace.strip('/')
    if result and not re.fullmatch(r'[A-Za-z_][\w]*(/[A-Za-z_][\w]*)*', result):
        raise ValueError(f'Invalid sentry namespace: {namespace!r}')
    return result


def topic(namespace, name):
    """Return an absolute ROS topic within the navigation namespace."""
    return '/' + '/'.join(filter(None, (normalized_namespace(namespace), name.strip('/'))))


def frame(namespace, name):
    """Prefix robot frames while keeping the shared map frame unchanged."""
    return name if name == 'map' else topic(namespace, name).lstrip('/')


def navigation_parameters(source, namespace, map_file, use_sim_time, bt_xml_dir='', scene=None):
    """Rewrite parameter values without changing parameter keys or recovery policy."""
    namespace = normalized_namespace(namespace)

    def rewrite(value, key=''):
        if isinstance(value, dict):
            return {k: rewrite(v, k) for k, v in value.items()}
        if isinstance(value, list):
            return [rewrite(item, key) for item in value]
        if key == 'use_sim_time':
            return use_sim_time
        if not isinstance(value, str):
            return value
        has_namespace_placeholder = '<robot_namespace>' in value
        value = value.replace('<robot_namespace>', namespace)
        if key in TOPIC_KEYS and has_namespace_placeholder:
            return '/' + value.lstrip('/')
        if key in FRAME_KEYS and value in ROBOT_FRAMES:
            return frame(namespace, value)
        if key in TOPIC_KEYS and value.startswith('/'):
            if value not in ('/clock', '/tf', '/tf_static'):
                return topic(namespace, value)
        if key in BT_KEYS and bt_xml_dir:
            filename = ('navigate_to_pose_w_replanning_and_recovery.xml'
                        if key == 'default_nav_to_pose_bt_xml' else
                        'navigate_through_poses_w_replanning_and_recovery.xml')
            return os.path.join(bt_xml_dir, filename)
        return value

    result = rewrite(deepcopy(source))
    result['map_server']['ros__parameters']['yaml_filename'] = map_file
    # A distinct input prevents the physical command bridge from bypassing or
    # feeding back into the same watchdog in the namespaced dual scene.
    result['fake_vel_transform']['ros__parameters']['output_cmd_vel_topic'] = topic(
        namespace, 'cmd_vel_chassis')
    if scene is not None:
        result['planner_server']['ros__parameters']['GridBased']['tunnel_cost_regions'] = (
            scene.planner_regions)
    return result


def sentry_bridge(template, sentry_name, namespace):
    """Use one bridge template, with robot sensors and joint state isolation."""
    return (template.replace('<sentry_name>', sentry_name)
            .replace('"/joint_states"', f'"{topic(namespace, "joint_states")}"'))


def sentry_model(template, sentry_name, namespace):
    """Change only transport names and the lidar frame, never model geometry."""
    return (template.replace('/sentry_robot1/gimbal_yaw_cmd',
                             f'/{sentry_name}/gimbal_yaw_cmd')
            .replace('<ignition_frame_id>lidar_link</ignition_frame_id>',
                     f'<ignition_frame_id>{frame(namespace, "lidar_link")}'
                     '</ignition_frame_id>'))


def behavior_tree_xml(template, namespace, tunnel_passage=True, use_sim_time=True,
                      exit_spin_speed=0.0, keep_exit_spin_at_goal=False, scene=None):
    """Resolve frame attributes without changing the shared recovery policy."""
    if not math.isfinite(exit_spin_speed):
        raise ValueError('exit_spin_speed must be finite (rad/s)')
    root = ET.fromstring(template)
    if not tunnel_passage:
        for parent in root.iter():
            for child in list(parent):
                if child.tag in {'TunnelFinalGoal', 'TunnelPassage', 'TunnelRouteGuard'}:
                    parent.remove(child)
    for node in root.iter():
        if scene is not None and node.tag in {'TunnelPassage', 'TunnelRouteGuard'}:
            node.set('tunnels', scene.bt_tunnels)
            node.set('tunnel_half_width', str(scene.tunnel_half_width))
        if node.tag == 'TunnelPassage':
            node.set('use_sim_time', str(use_sim_time).lower())
            # A single RViz goal never inherits fixed-route automatic spin.
            if node.get('goals') is not None:
                node.set('exit_spin_speed', str(float(exit_spin_speed)))
                node.set('keep_exit_spin_at_goal', str(keep_exit_spin_at_goal).lower())
        for key in ('robot_base_frame', 'global_frame', 'body_frame', 'control_frame'):
            if node.get(key) in ROBOT_FRAMES:
                node.set(key, frame(namespace, node.get(key)))
    return ET.tostring(root, encoding='unicode')


def gazebo_resource_path(bringup_share, description_share, simulator_share, existing=''):
    """Build the model search path for the simulation scene."""
    paths = [os.path.join(bringup_share, 'models', 'vendor')]
    paths.extend(filter(None, existing.split(os.pathsep)))
    paths.extend([os.path.join(description_share, 'resource', 'models'),
                  os.path.join(simulator_share, 'resource', 'models')])
    return os.pathsep.join(dict.fromkeys(paths))


def finite_pose(values, name):
    """Validate one XYZYaw launch pose before creating any ROS processes."""
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ValueError(name + ' must contain four finite XYZYaw values') from error
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError(name + ' must contain four finite XYZYaw values')
    return result
