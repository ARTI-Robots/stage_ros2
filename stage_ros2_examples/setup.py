from glob import glob
from setuptools import setup

package_name = 'stage_ros2_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Buchegger',
    maintainer_email='a.buchegger@arti-robots.com',
    description='Examples of using stage_ros2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'factory_control = ' + package_name + '.factory_control:main'
        ],
    },
)
