from setuptools import find_packages, setup

package_name = 'moali_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/move_square_launch.py']),
        ('share/' + package_name, ['launch/square_distance_launch.py']),
        ('share/' + package_name, ['launch/convert_launch.py']),
        ('share/' + package_name, ['launch/FSM_launch.py']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moham',
    maintainer_email='moham@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moves_square = moali_pkg.move_square:main',
            'square_distance = moali_pkg.square_distance:main',
            'convert = moali_pkg.convert:main',
            'FSM = moali_pkg.FSM:main',
            'Mode = moali_pkg.Mode:main',
            'Estop = moali_pkg.Estop:main',
        ],
    },
)