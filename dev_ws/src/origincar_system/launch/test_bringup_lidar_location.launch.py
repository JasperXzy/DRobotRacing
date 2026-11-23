"""Competition with lidar scan matching."""

from origincar_system.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(lidar_localization=True, competition=True)
