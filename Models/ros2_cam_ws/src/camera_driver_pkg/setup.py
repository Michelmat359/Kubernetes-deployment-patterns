from setuptools import setup
package_name = 'camera_driver_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Angel Mateo-Casali',
    maintainer_email='mmateo@cigip.upv.es',
    description='ROS 2 camera driver publishing /camera/image_raw - CIGIP',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_driver = camera_driver_pkg.driver:main',
        ],
    },
)
