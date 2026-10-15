from setuptools import find_packages, setup

package_name = 'tank01_safety'

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
    description='Tank01 safety supervisor, geofence and geodesy helpers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor = tank01_safety.safety_supervisor:main',
        ],
    },
)
