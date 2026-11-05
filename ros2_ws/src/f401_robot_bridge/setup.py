from setuptools import setup

package_name = 'f401_robot_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot project',
    maintainer_email='robot@example.invalid',
    description='STM32 base bridge over Mac gateway',
    license='Proprietary',
    entry_points={'console_scripts': [
        'network_bridge = f401_robot_bridge.bridge_node:main',
        'keyboard = f401_robot_bridge.keyboard_node:main',
        'keyboard_window = f401_robot_bridge.keyboard_window:main',
    ]},
)
