from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'origincar_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.yaml') + glob('map/*.pgm') + glob('map/*.png')),
        (os.path.join('share', package_name, 'map', 'serialized'), glob('map/serialized/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'config'),
            glob('origincar_nav2/recorded_waypoints.yaml')),
        # 比赛航点文件 — 放在包根目录，cruiser节点直接读取
        (os.path.join('share', package_name),
            glob('origincar_nav2/waypoints_*.yaml') +
            glob('origincar_nav2/ysvv_waypoints.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='jasper.zhengyi.xu@gmail.com',
    description='Origincar navigation bringup with obstacle avoidance cruise',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = origincar_nav2.twist_to_ackermann:main',
            'waypoint_cruiser = origincar_nav2.waypoint_cruiser:main',
            'fixed_waypoint_demo = origincar_nav2.fixed_waypoint_demo:main',
            'waypoint_recorder = origincar_nav2.waypoint_recorder:main',
            'record_ysvv_waypoints = origincar_nav2.record_ysvv_waypoints:main',
            'nav2_ysvv_waypoints = origincar_nav2.ysvv_waypoint_navigator:main',
            'map_overlay_publisher = origincar_nav2.map_overlay_publisher:main',
            'trigger_cruise = origincar_nav2.trigger_cruise:main',
            'cmd_vel_kp_scaler = origincar_nav2.cmd_vel_kp_scaler:main',
        ],
    },
)
