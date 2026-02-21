from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control_module'

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
    maintainer='dieguin',
    maintainer_email='dieguin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dc_motor = motor_control_module.dc_motor:main',
            'set_point = motor_control_module.set_point:main',
            'controller = motor_control_module.controller:main',
            'server = motor_control_module.server:main'
        ],
    },
)

