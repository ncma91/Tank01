from setuptools import find_packages, setup

package_name = 'tank01_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Nachbaur',
    maintainer_email='martin.nachbaur@gmx.at',
    description='Tank01 tracked base driver (kinematics, odometry, CAN bridge)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = tank01_base.base_node:main',
        ],
    },
)
