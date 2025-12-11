from setuptools import find_packages, setup
package_name = 'abot_01_pkg'
setup(
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
       ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
       ('share/' + package_name,
        ['package.xml']),
       # install launch file(s)
       ('share/' + package_name + '/launch', [
           'launch/follow_ball_launch.py',
       ]),
   ],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='student',
   maintainer_email='student@example.com',
   description='FSM + PID controller to make abot-01 follow a blue pingpong ball.',
   license='TODO',
   tests_require=['pytest'],
   entry_points={
      'console_scripts': [
          'fsm = abot_01_pkg.FSM:main',
          'mode = abot_01_pkg.Mode:main',
          'follow_ball_controller = abot_01_pkg.follow_ball_controller:main',
          'filter = abot_01_pkg.filter:main',
          'image_info = abot_01_pkg.image_info:main',
      ],
   },
)
