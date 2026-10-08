from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files for quick launching
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'calib'), glob('calib/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dylan-work',
    maintainer_email='dylanfanner@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_home_commander = utils.vehicle_home_commander:main',
            'tag_gps_groundtruth_local_map = utils.tag_gps_px4_local_map:main',
            'generic_gps_transformer = utils.gps_to_px4_local_map:main',
	    'ublox_gps_node = utils.ublox_node:main',
        ],
    },
)
