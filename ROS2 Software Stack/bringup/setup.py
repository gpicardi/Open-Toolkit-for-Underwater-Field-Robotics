from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
          ('share/ament_index/resource_index/packages',
              ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))), # include launch files
      ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='silvercore',
    maintainer_email='jaguirregomezcorta@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
