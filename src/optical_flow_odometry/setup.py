from setuptools import find_packages, setup

package_name = 'optical_flow_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozodbek',
    maintainer_email='179662970+Ozodbek-create@users.noreply.github.com',
    description='Camera optical-flow odometry with metric geometry and tracking diagnostics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'optical_flow_node = optical_flow_odometry.optical_flow_node:main',
            'metric_odometry_node = optical_flow_odometry.metric_odometry_node:main',
        ],
    },
)
