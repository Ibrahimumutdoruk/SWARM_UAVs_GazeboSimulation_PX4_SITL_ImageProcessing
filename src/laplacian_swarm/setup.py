from setuptools import setup
import os
from glob import glob

package_name = 'laplacian_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laplacian',
    maintainer_email='dev@laplacian.local',
    description='Decentralized swarm agents over PX4 Offboard (uXRCE-DDS)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_agent_node = laplacian_swarm.swarm_agent_node:main',
            'px4_gateway_node = laplacian_swarm.px4_gateway_node:main',
            'localization_node = laplacian_swarm.localization_node:main',
            'vision_node = laplacian_swarm.vision_node:main',
            'mission_trigger = laplacian_swarm.mission_trigger:main',
        ],
    },
)
