from setuptools import find_packages, setup

package_name = 'dualsense_teleop'

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
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dualsense_publish_tf = dualsense_teleop.dualsense_publish_tf:main',
            'dualsense_publish_imu = dualsense_teleop.dualsense_publish_imu:main',
        ],
    },
)
