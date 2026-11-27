from setuptools import setup
from glob import glob
import os

package_name = 'spider_sim'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # FIX: these were missing, so `ros2 launch` could not find any of the nodes.
    entry_points={
        'console_scripts': [
            'gait_controller = spider_sim.gait_controller:main',
            'arm_ik = spider_sim.arm_controller:main',
            'gripper_controller = spider_sim.gripper_controller:main',
            'mission_manager = spider_sim.mission_manager:main',
            'demo = spider_sim.demo:main',
        ],
    },
)
