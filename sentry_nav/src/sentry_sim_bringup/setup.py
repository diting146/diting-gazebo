import os
from glob import glob

from setuptools import setup

package_name = 'sentry_sim_bringup'


def recursive_data_files(directory):
    """Return (install_dir, files) tuples keeping the relative layout."""
    grouped = {}
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if not os.path.isfile(path):
            continue
        install_dir = os.path.join('share', package_name, os.path.dirname(path))
        grouped.setdefault(install_dir, []).append(path)
    return sorted(grouped.items())


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
]
data_files.extend(recursive_data_files('models'))
data_files.extend(recursive_data_files('maps'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diting',
    maintainer_email='diting@example.com',
    description='RM25-model sentry simulation bringup and sensor bridges.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_lio_gateway = sentry_sim_bringup.sim_lio_gateway:main',
            'sensor_gimbal_world_lock = sentry_sim_bringup.sensor_gimbal_world_lock:main',
            'livox_cloud_adapter = sentry_sim_bringup.livox_cloud_adapter:main',
            'localization_error_monitor = sentry_sim_bringup.localization_error_monitor:main',
            'sim_reference_pose = sentry_sim_bringup.sim_reference_pose:main',
            'sim_serial_gateway = '
            'sentry_sim_bringup.sim_serial_gateway:main',
            'sim_ground_truth_odometry = '
            'sentry_sim_bringup.sim_ground_truth_odometry:main',
            'sentry_gimbal_tf = '
            'sentry_sim_bringup.sentry_gimbal_tf:main',
            'gimbal_world_lock = '
            'sentry_sim_bringup.gimbal_world_lock:main',
        ],
    },
)
