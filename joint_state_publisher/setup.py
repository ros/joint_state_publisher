from setuptools import find_packages
from setuptools import setup

package_name = 'joint_state_publisher'

setup(
    name=package_name,
    version='2.4.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='David V. Lu!!',
    author_email='davidvlu@gmail.com',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development',
    ],
    description=(
        'A python node to publish `sensor_msgs/JointState` messages for a '
        'robot described with URDF.'
    ),
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher.joint_state_publisher:main',
        ],
    },
)
