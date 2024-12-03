from setuptools import setup
import os
from glob import glob

package_name = 'tractor_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add the launch directory for installation
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Add URDF directory
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charles M. Raines',
    maintainer_email='cmr750@msstate.edu',
    description='Tractor URDF Description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your Python executables here if any
        ],
    },
)
