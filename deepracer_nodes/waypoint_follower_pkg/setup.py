from setuptools import setup
import os
#from glob import glob

package_name = 'waypoint_follower_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/waypoint_follower_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Narella Cerquetti',
    maintainer_email='narella.cerquetti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower_node = waypoint_follower_pkg.waypoint_follower_node:main'
        ],
    },
)
