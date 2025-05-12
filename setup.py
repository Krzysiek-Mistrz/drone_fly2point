from setuptools import setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Ustawienie ścieżki do folderu launch
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krzychu',
    maintainer_email='krzychu@example.com',
    description='Opis twojego pakietu',
    license='Licencja',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control_node = drone_control.drone_control_node:main'
        ],
    },
)
