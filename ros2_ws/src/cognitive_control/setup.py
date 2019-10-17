from setuptools import find_packages
from setuptools import setup

package_name = 'cognitive_control'

setup(
    name=package_name,
    version='0.2.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'cellebrum = cognitive_control.scripts.cellebrum:main'
        ],
    },
)
