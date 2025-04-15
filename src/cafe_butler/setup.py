from setuptools import setup
import os
from glob import glob

package_name = 'cafe_butler'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Butler Developer',
    maintainer_email='developer@example.com',
    description='Butler robot for French Door Cafe',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'butler_server = cafe_butler.butler_server:main',
            'butler_client = cafe_butler.butler_client:main',
        ],
    },
)

