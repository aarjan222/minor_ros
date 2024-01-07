from setuptools import find_packages, setup

package_name = 'pubsub'

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
    maintainer='aarjan',
    maintainer_email='077bei004.aarjan@pcampus.edu.np',
    description='Simple TurtleBot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pubsub.vel_publisher:main',
            'listener = pubsub.vel_subscriber:main',
            'draw_circle = pubsub.draw_circle:main',
            'test_node = pubsub.mynode:main',
            'turtle_controller = pubsub.turtle_controller:main',
            'motor_vel = pubsub.my_vel_subscriber:main',
        ],
    },
)
