from setuptools import setup

# colcon通过ament_python调用setuptools，安装Python包及ros2 run可执行入口。
package_name = 'f401_robot_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # 空resource标记登记到ament索引；package.xml携带包名、依赖和构建类型。
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
    # 左边是ros2 run使用的名称，右边是Python模块路径:入口函数。
    # 例如ros2 run f401_robot_bridge network_bridge最终调用bridge_node.py的main()。
    # 入口名不等于ROS节点名；节点名由Node构造函数中的字符串决定。
    entry_points={'console_scripts': [
        'network_bridge = f401_robot_bridge.bridge_node:main',
        'keyboard = f401_robot_bridge.keyboard_node:main',
        'keyboard_window = f401_robot_bridge.keyboard_window:main',
    ]},
)
