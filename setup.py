from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evan',
    maintainer_email='evangoldman11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = turtlebot_teleop.keyboard_teleop:main',
            'slam_self = turtlebot_teleop.slam_self:main',
            'navigator = turtlebot_teleop.navigator:main',
        ],
    },
)
