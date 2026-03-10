from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpc_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
      (os.path.join('share',package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar',
    maintainer_email='ofc1227@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'path_drawer = mpc_turtlebot.path_drawer:main',
        'mpc_controller_casadi = mpc_turtlebot.mpc_controller_casadi:main',
        'path_mpc_casadi = mpc_turtlebot.path_mpc_casadi:main',
    ],
    },
)
