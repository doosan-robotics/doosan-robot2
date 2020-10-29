from setuptools import setup

package_name = 'dsr_example2_py'

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
    maintainer='DoosanRobotics',
    maintainer_email='ros.robotics@doosan.com',
    description='The dsr_example2 Python package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'dsr_service_motion_basic        = dsr_example2_py.dsr_service_motion_basic:main',
	        'dsr_service_motion_simple       = dsr_example2_py.dsr_service_motion_simple:main',
	        'dsr_service_motion_simple_class = dsr_example2_py.dsr_service_motion_simple_class:main',
        ],
    },
)
