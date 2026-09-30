import os
from glob import glob
from setuptools import setup

package_name = 'paxini_tactile'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Paxini Tactile & Amazing Hand Integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'paxini_node = paxini_tactile.paxini_node:main',
            'amazing_hand_node = paxini_tactile.hand_driver_node:main',
            'grasp_controller_node = paxini_tactile.grasp_controller_node:main',
        ],
    },
)
