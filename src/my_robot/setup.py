from setuptools import setup, find_packages

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/model', ['model/Differential_bot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrit',
    maintainer_email='amrit@example.com',
    description='Test Robot Teleop',
    license='Apache2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = my_robot.scripts.teleop:main',
            'apfa = my_robot.scripts.apfa:main',
            'control = my_robot.scripts.control:main',
            'test_bed = my_robot.scripts.test_bed:main',
            'dwa = my_robot.scripts.dwa:main',
            'dwam = my_robot.scripts.dwam:main',
        ],
    },
)
