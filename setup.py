from setuptools import find_packages, setup

package_name = 'aiddl_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uekn',
    maintainer_email='uwe.kockemann@oru.se',
    description='Collection of AIDDL-ROS nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sender = aiddl_ros2_bridge.sender:main',
            'receiver = aiddl_ros2_bridge.receiver:main', 
            'actor = aiddl_ros2_bridge.actor:main',
            'service = aiddl_ros2_bridge.service:main'
        ],
    },
)
