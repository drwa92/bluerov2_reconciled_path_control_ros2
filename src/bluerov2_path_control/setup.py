from setuptools import setup
from glob import glob
import os

package_name = 'bluerov2_path_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Waseem Akram',
    maintainer_email='drwa92@gmail.com',
    description='Direct-thruster BlueROV2 path-following controller with mission services.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_controller = bluerov2_path_control.path_controller_node:main',
            'mission_client = bluerov2_path_control.mission_client:main',
        ],
    },
)
