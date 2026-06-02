from setuptools import find_packages, setup

package_name = 'archer_yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boss Aditya',
    maintainer_email='archer@ros.local',
    description='Archer YOLOv8 vision node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = archer_yolo.yolo_node:main',
        ],
    },
)
