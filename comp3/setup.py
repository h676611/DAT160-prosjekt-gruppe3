from setuptools import setup
import os 
from glob import glob  

package_name = 'comp3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='142395938+h676611@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comp3 = comp3.comp3:main',
            'bug2_controller = comp3.bug2_controller:main',
            'wallfollower_controller = comp3.wallfollower_controller:main',
            'gotopoint_controller = comp3.gotopoint_controller:main',
            'robot_controller = comp3.robot_controller:main',
            'wall_classifier = comp3.wall_classifier:main',
            'leader = comp3.leader:main',
            'marker_detection = comp3.marker_detection:main',
        ],
    },
)
