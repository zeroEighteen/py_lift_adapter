from setuptools import find_packages, setup

package_name = 'ros2_lift_adapter'

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
    maintainer='dsta-intern-fleet',
    maintainer_email='dsta-intern-fleet@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "listener = ros2_lift_adapter.listener:main",
	    "ros2_sim_gui = ros2_lift_adapter.ros2_sim_gui:main",
	    "ros2_sim_handler = ros2_lift_adapter.ros2_sim_handler:main",
	    "lift_adapter_sim = ros2_lift_adapter.lift_adapter_sim:main",
	    "fleet_adapter = ros2_lift_adapter.fleet_adapter:main"
        ],
    },
)
