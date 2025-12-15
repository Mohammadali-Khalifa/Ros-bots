from setuptools import setup
import os
from glob import glob
package_name = 'Final_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],   # <-- FORCE include the python module
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Final_pkg',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm = Final_pkg.FSM:main',
            'mode = Final_pkg.Mode:main',
            'filter = Final_pkg.filter:main',
            'image_info = Final_pkg.image_info:main',
            'follow_ball_controller = Final_pkg.follow_ball_controller:main',
            'gripper_keyboard = Final_pkg.gripper_keyboard_node:main',
            'gripper_serial = final_pkg.gripper_serial_node:main',
        ],
    },
)
