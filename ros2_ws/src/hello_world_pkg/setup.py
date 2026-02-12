from setuptools import setup

package_name = 'hello_world_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='A simple ROS2 hello world package with publisher and subscriber nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = hello_world_pkg.publisher_node:main',
            'subscriber = hello_world_pkg.subscriber_node:main',
        ],
    },
)
