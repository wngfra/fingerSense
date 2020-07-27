from setuptools import setup

package_name = 'finger_sense'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wngfra',
    maintainer_email='wngfra@gmail.com',
    description='A ROS2 package for tactile perception using FPM-finger sensor',
    license='Apache-2.0 License ',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tactile_percept = finger_sense.tactile_percept:main'
        ],
    },
)
