from setuptools import find_packages, setup

package_name = 'pickup_delivery_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pd_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudhi',
    maintainer_email='sudhi@todo.todo',
    description='Pickup & delivery using HSV markers + FSM + Arduino gripper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsv_detector_node = pickup_delivery_pkg.hsv_detector_node:main',
            'pickup_delivery_fsm_node = pickup_delivery_pkg.pd_fsm_node:main',
            'gripper_node = pickup_delivery_pkg.gripper_node:main',
            'mode_input_node = pickup_delivery_pkg.mode_input_node:main',
            'gripper_teleop_node = pickup_delivery_pkg.gripper_teleop_node:main',
        ],
    },
)
