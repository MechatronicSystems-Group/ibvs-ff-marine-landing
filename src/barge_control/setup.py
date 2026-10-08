import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'barge_control'

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
        # Include resource files
        (os.path.join('share', package_name, 'resource'), ['resource/baseline.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dylan-work',
    maintainer_email='dylanfanner@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barge_pohang_node = barge_control.barge_pohang:main',
            'barge_linear_node = barge_control.barge_linear:main'
        ],
    },
)
