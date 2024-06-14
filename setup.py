from setuptools import setup

package_name = 'mae148_team8_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'hitch_node = mae148_team8_pkg.hitch_node:main',
        'dispense_node = mae148_team8_pkg.dispense_node:main',
        'forward_node = mae148_team8_pkg.forward_node:main'
        'gps_node = mae148_team8_pkg.gps_node:main'
        ],
    },
)
