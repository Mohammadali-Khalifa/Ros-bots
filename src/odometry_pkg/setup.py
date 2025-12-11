from setuptools import find_packages, setup

package_name = 'odometry_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_move_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moham',
    maintainer_email='moham@todo.todo',
    description='odometry student pkg',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'robot_move = odometry_pkg.robot_move:main',
        ],
    },
)
