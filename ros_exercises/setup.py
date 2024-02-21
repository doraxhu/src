from setuptools import setup

package_name = 'ros_exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='dorah@mit.edu',
    description='Description: minimal publisher/subscriber using rclpy',
    license='TODO: License declaration (idk)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = ros_exercises.my_node:main',
	    'talker = ros_exercises.publisher_member_function:main',
	    'listener = ros_exercises.subscriber_member_function:main',
	    'simple_publisher = ros_exercises.simple_publisher:main',
	    'simple_subscriber = ros_exercises.simple_subscriber:main',
	    'complex_publisher = ros_exercises.complex_publisher:main'
        ],
    },
)
