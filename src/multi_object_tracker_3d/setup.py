from setuptools import find_packages, setup

package_name = 'multi_object_tracker_3d'

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
    maintainer='rae384',
    maintainer_email='rae3840924@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_object_tracker_3d_node = multi_object_tracker_3d.mot3d_node:main'
        ],
    },
)
