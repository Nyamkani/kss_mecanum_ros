from setuptools import find_packages, setup

package_name = 'mecanum_gateway'

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
    maintainer='Seong Sik, Kim',
    maintainer_email='george@studio3s.co.kr',
    description='TCP JSON gateway for mecanum robot status and commands.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'gateway_node = mecanum_gateway.gateway_node:main',
    ]},
)
