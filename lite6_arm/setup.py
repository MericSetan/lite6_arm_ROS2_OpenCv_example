from setuptools import find_packages, setup

package_name = 'lite6_arm'

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
    maintainer='meric',
    maintainer_email='meric@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'initialize_robot = lite6_arm.initialize_robot:main',
        'robot_control = lite6_arm.robot_control:main',
        'find_boxes_server = lite6_arm.find_boxes_server:main'
        ],
    },
)
