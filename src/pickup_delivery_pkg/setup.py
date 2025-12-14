from setuptools import find_packages, setup

package_name = 'pickup_delivery_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pickup_delivery_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudhi',
    maintainer_email='sudhi@todo.todo',
    description='Pickup & delivery using HSV markers + FSM',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'multi_color_detector_node = pickup_delivery_pkg.multi_color_detector_node:main',
            'pickup_delivery_fsm_node = pickup_delivery_pkg.pickup_delivery_fsm_node:main',
        ],
    },
)
