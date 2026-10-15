from glob import glob

from setuptools import find_packages, setup

package_name = 'tank01_mower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Nachbaur',
    maintainer_email='martin.nachbaur@gmx.at',
    description='Tank01 coverage planning and mowing missions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = tank01_mower.mission_node:main',
            'record_boundary = tank01_mower.record_boundary:main',
            'plan_coverage = tank01_mower.coverage:main',
        ],
    },
)
