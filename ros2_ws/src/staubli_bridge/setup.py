from setuptools import setup

package_name = 'staubli_bridge'

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
    maintainer='todo',
    maintainer_email='todo@mail.com',
    description='Bridge TCP bidirectionnel entre le robot Stäubli et ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge = staubli_bridge.staubli_live_bridge:main',
        ],
    },
)
