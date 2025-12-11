from setuptools import find_packages, setup

package_name = 'color_filter_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/color_filter_launch.py']),
        ('share/' + package_name + '/images/blue_pingpong', [
            'images/blue_pingpong/image_blue_0.png',
            'images/blue_pingpong/image_blue_1.png',
            'images/blue_pingpong/image_blue_2.png',
            'images/blue_pingpong/image_blue_3.png',
            'images/blue_pingpong/image_blue_4.png',
            'images/blue_pingpong/image_blue_5.png',
        ]),
        ('share/' + package_name + '/images/pink_pingpong', [
            'images/pink_pingpong/image_pink_0.png',
            'images/pink_pingpong/image_pink_1.png',
            'images/pink_pingpong/image_pink_2.png',
            'images/pink_pingpong/image_pink_3.png',
            'images/pink_pingpong/image_pink_4.png',
            'images/pink_pingpong/image_pink_5.png',
        ]),
        ('share/' + package_name + '/images/white_pingpong', [
            'images/white_pingpong/image_white_0.png',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moham',
    maintainer_email='moham@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'filter = color_filter_pkg.filter:main',
            'image_info = color_filter_pkg.image_info:main',
        ],
    },
)

