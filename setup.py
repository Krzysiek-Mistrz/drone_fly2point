from setuptools import setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # setting paths
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krzychu',
    maintainer_email='krzyskuar@gmail.com',
    description='just flying 2 certain point in space',
    license='GNU GPL V3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #node_name = package_name.node_name when simple prj usually package_name == node_name ;)
            'drone_fly_to_point = drone_fly_to_point.drone_fly_to_point:main' 
        ],
    },
)
