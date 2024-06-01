from setuptools import setup

package_name = 'coppeliasim_ros2_tools'

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
    maintainer='BenBlop',
    maintainer_email='benblop@gmail.com',
    description='Tools to work with ROS2 in CoppeliaSim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coppeliasim_ros2_interface = coppeliasim_ros2_tools.coppeliasim_ros2_interface:main'
        ],
    },
)
