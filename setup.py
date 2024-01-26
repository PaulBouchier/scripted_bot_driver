import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scripted_bot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Bouchier',
    maintainer_email='paul.bouchier@gmail.com',
    description='Scripts for driving a generic bot from a motion-file',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_bot = scripted_bot_driver.move_bot:main',
            'drive_straight_odom = scripted_bot_driver.drive_straight_odom:main',
            'stop = scripted_bot_driver.stop:main',
            'move_parent = scripted_bot_driver.move_parent:main',
            'scripted_mover = scripted_bot_driver.scripted_mover:main',
        ],
    },
)
