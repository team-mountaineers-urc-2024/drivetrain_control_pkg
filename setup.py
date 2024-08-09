import os 
from glob import glob # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
from setuptools import find_packages, setup # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

package_name = 'drivebase_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'drivebase_control_node = drivebase_control_pkg.drivebase_control_node:main',
            'drivebase_control_node_w_srv = drivebase_control_pkg.drivebase_control_node_w_srv:main',

            'drivebase_odometry_node = drivebase_control_pkg.drivebase_odometry_node:main',
        ],
    },
)
