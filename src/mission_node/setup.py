from setuptools import setup

package_name = 'mission_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Mission Node for Visual Odometry Drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mission_node.mission_node:main',
            'odom_emulator = mission_node.odom_emulator:main',
        ],
    },
)
