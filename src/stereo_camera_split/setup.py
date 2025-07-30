from setuptools import find_packages, setup

package_name = 'stereo_camera_split'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/split_camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='golftrolley',
    maintainer_email='Wilsonyew38@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_split_camera_node = stereo_camera_split.stereo_camera_split_node:main',
        ],
    },
)
