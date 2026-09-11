from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ugv_gazebo'


def files_under(directory):
    entries = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        destination = os.path.join('share', package_name, root)
        entries.append((destination, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/trajectories', glob('trajectories/*.yaml')),
        ('share/' + package_name + '/trajectories/dense', glob('trajectories/dense/*.yaml')),
    ] + files_under('models'),
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='batbaina',
    maintainer_email='ericbabaina@gmail.com',
    description='Gazebo execution and validation of SHA*/LCBR Ackermann UGV trajectories.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
            'multi_ugv_tracker = ugv_gazebo.multi_ugv_tracker:main',
        'pure_pursuit_tracker = ugv_gazebo.pure_pursuit_tracker:main',
        'primitive_executor = ugv_gazebo.primitive_executor:main',
        'lcbr_tracker = ugv_gazebo.lcbr_tracker:main',
        'world_pose_monitor = ugv_gazebo.world_pose_monitor:main',
        'trajectory_tracker = ugv_gazebo.trajectory_tracker:main',
    ]},
)
