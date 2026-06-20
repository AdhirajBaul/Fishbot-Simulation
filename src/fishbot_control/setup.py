from setuptools import find_packages, setup

package_name = 'fishbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adhiraj',
    maintainer_email='adhiraj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'swim_publisher = fishbot_control.swim_command_publisher:main',
            'swim_listener = fishbot_control.swim_command_listener:main',
            'fish_swimmer = fishbot_control.fish_swimmer:main',
            'fish_teleop = fishbot_control.fish_teleop:main',
        ],
    },
)