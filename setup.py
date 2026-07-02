from setuptools import find_packages, setup

package_name = 'mission_control_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay12',
    maintainer_email='f20250598@goa.bits-pilani.ac.in',
    description=(
        'Mission Control GUI: PyQt5 dashboard with an embedded ROS 2 '
        'Nav2 action client and live telemetry, for Project Kratos.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = mission_control_gui.main:main',
        ],
    },
)
