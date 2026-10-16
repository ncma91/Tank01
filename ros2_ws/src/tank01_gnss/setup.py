from setuptools import find_packages, setup

package_name = 'tank01_gnss'

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
    description='Unicore UM982 dual-antenna RTK driver for Tank01',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_node = tank01_gnss.um982_node:main',
        ],
    },
)
