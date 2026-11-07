from glob import glob
from setuptools import find_packages, setup

package_name = 'lab2_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ROS 2 Lab 2 service implementation.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'service_server = lab2_controller.service_nodes:service_server_main',
        'service_client = lab2_controller.service_nodes:service_client_main',
    ]},
)
