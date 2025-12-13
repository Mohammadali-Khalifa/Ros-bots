from setuptools import find_packages, setup

package_name = 'Ball_follow_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab3_3_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudhi',
    maintainer_email='sudhi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'mode_switcher_node = lab3_3.mode_switcher_node:main',
        'mode_input_node = lab3_3.mode_input_node:main',
        'ball_follow_controller_node = lab3_3.ball_follow_controller_node:main',
        ],
    },
)