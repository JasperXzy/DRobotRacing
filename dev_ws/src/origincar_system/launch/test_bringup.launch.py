"""Competition with AMCL localization."""

from origincar_system.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(lidar_localization=False, competition=True)
