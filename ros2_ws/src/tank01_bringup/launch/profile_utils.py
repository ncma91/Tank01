"""Helpers shared by the Tank01 launch files: parameter profiles.

A profile (e.g. "prototyp") is a folder config/profiles/<name>/ with YAML files that are
loaded after the file of the same name in config/ and override only the keys they set.
"""

import os

from ament_index_python.packages import get_package_share_directory

CONFIG_DIR = os.path.join(get_package_share_directory('tank01_bringup'), 'config')

# Defaults that a profile implies for launch arguments (can still be overridden).
PROFILE_LAUNCH_DEFAULTS = {
    '': {'variant': 'full', 'gnss_driver': 'ublox', 'lidar_driver': 'ouster',
         'imu_driver': 'xsens'},
    'prototyp': {'variant': 'prototyp', 'gnss_driver': 'um982', 'lidar_driver': 'unitree',
                 'imu_driver': 'lidar'},
}


def param_files(profile: str, name: str) -> list:
    """config/<name> plus config/profiles/<profile>/<name> if that override exists."""
    files = [os.path.join(CONFIG_DIR, name)]
    if profile:
        override = os.path.join(CONFIG_DIR, 'profiles', profile, name)
        if os.path.exists(override):
            files.append(override)
    return files


def profile_default(profile: str, key: str, given: str) -> str:
    """Explicit launch argument wins; 'auto' falls back to the profile default."""
    if given and given != 'auto':
        return given
    if profile not in PROFILE_LAUNCH_DEFAULTS:
        raise ValueError(f'unknown profile {profile!r}, known: {list(PROFILE_LAUNCH_DEFAULTS)}')
    return PROFILE_LAUNCH_DEFAULTS[profile][key]
