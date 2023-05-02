from setuptools import setup

package_name = 'bno055_raspi_squeeze'

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
    maintainer='napster',
    maintainer_email='aravindadhith@asu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'imu_node = bno055_raspi_squeeze.imu_4_timesync:main'
        
        ],
    },
)
