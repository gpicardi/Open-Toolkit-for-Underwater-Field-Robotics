from setuptools import find_packages, setup

package_name = 'ms8607'

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
    maintainer='silvercore',
    maintainer_email='silvercore@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ms8607_i2c0 = ms8607.ms8607_i2c0:main',
            'ms8607_i2c1 = ms8607.ms8607_i2c1:main',
            'ms8607 = ms8607.ms8607:main'
        ],
    },
)
