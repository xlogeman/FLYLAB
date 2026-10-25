from setuptools import find_packages, setup

package_name = 'flylab_gcs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flylab',
    maintainer_email='flylab@todo.todo',
    description='flylab camera viewer: pull AirSim camera images via RPC and display',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_view = flylab_gcs.camera_view:main',
            'ground_station = flylab_gcs.ground_station:main',
        ],
    },
)
