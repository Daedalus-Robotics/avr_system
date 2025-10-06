from setuptools import find_packages, setup

package_name = 'avr_system'

setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Hunter Baker',
        maintainer_email='hunterbaker@me.com',
        description='A node to control general system functions',
        license='LGPL-3.0-only',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'system_control_node = avr_system.system_node:main',
            ],
        },
)
