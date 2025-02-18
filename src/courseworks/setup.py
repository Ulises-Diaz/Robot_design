from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'courseworks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Agrega esta línea para incluir el archivo launcher.py
        ('share/' + package_name + '/launch', glob(os.path.join('launch', 'launcher.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uli',
    maintainer_email='A00837223@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = courseworks.publisher:main',  
            'subscriber = courseworks.subscriber:main',   
        ],
    },
)
