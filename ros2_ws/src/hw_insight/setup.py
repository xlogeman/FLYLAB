import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'hw_insight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'yolo'), glob('yolo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hw',
    maintainer_email='toplaya@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard = hw_insight.offboard:main',
            'decode_px4_fmu_out_vehicle_status = hw_insight.msg_px4_fmu_out_vehicle_status:main',
            'keyboard_position = hw_insight.keyboard_position:main',
            'move_position = hw_insight.move_position:main',
            'keyboard_velocity = hw_insight.keyboard_velocity:main',
            'move_velocity = hw_insight.move_velocity:main',
            'multi_move_velocity = hw_insight.multi_move_velocity:main',
            'octomap_dynamic = hw_insight.octomap_dynamic:main',
            'lesson6_yolo = hw_insight.lesson6_yolo:main',
            'lesson6_track = hw_insight.lesson6_track:main',
            'rk3588_yolo = hw_insight.rk3588_yolo:main',
            'lesson8_vins_switch=hw_insight.lesson8_vins_switch:main',
            'lesson9_airsim_ros2_topic=hw_insight.lesson9_airsim_ros2_topic:main',
            'lesson9_only_imu=hw_insight.lesson9_only_imu:main',
            'lesson12_px4_ego_tracker=hw_insight.lesson12_px4_ego_tracker:main',
            'lesson13_px4_ego_tracker=hw_insight.lesson13_px4_ego_tracker:main',
        ],
    },
)
