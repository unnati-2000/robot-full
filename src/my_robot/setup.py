from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'move_robot = my_robot.ROS2moveforward:main',
            'test_arm = my_robot.test_arm_movement:main',
            'test_arm_action = my_robot.test_arm_action:main',
            'test_arm_raw = my_robot.test_arm_action_raw:main',
            'test_arm_rad = my_robot.test_arm_action_rad:main',
            'test_direct_servo = my_robot.test_direct_servo:main',
            'arm_controller = my_robot.arm_controller:main',
            'arm_and_movement = my_robot.arm_and_movement:main',
            'camera_node = my_robot.camera_node:main',
        ],
    },
)
