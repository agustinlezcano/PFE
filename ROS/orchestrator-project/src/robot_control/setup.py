from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='agustin',
    maintainer_email='agustin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = robot_control.publisher:main',
		'listener = robot_control.subscriber:main',
		'talker_ui = robot_control.main_with_ui:main',
		'robot_ui = robot_control.robot_ui_node:main',
		'trajectory_planner = robot_control.utils.trajectory_planner:main',
        ],
    },
)
