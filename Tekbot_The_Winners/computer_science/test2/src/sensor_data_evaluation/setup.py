from setuptools import find_packages, setup

package_name = 'sensor_data_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='foumi',
    maintainer_email='marzoukathagbodja@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_data_evaluation.sensor_publisher:main',
            'sensor_subscriber = sensor_data_evaluation.sensor_subscriber:main',
            'gui_subscriber = sensor_data_evaluation.gui_subscriber:main',
        ],
    },
)
