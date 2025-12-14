from setuptools import find_packages, setup

package_name = 'gripper_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gripper_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudhi',
    maintainer_email='sudhi@todo.todo',
    description='Gripper control (Arduino serial)',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gripper_serial_node = gripper_control_pkg.gripper_serial_node:main',
            'gripper_keyboard_node = gripper_control_pkg.gripper_keyboard_node:main',
        ],
    },
)
