from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Install model files
        (os.path.join('share', package_name, 'model'), glob('model/*.xacro')),
        (os.path.join('share', package_name, 'model'), glob('model/*.gazebo')),
        (os.path.join('share', package_name, 'model'), glob('model/*.macro')),
        (os.path.join('share', package_name, 'model'), glob('model/*.urdf')),

        # Install parameters
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*')),

        # >>>>>>>>>>>>>>>>> ADD THIS <<<<<<<<<<<<<<<<<<
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # Ensure all .py files in launch/ are copied
    
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adharsh',
    maintainer_email='adharsh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'y_tracker = mobile_robot.y_tracker_controller:main',
            'x_tracker = mobile_robot.x_tracker_controller:main',
            'robot_chat = mobile_robot.robot_chat:main'
        ],
    },
)
