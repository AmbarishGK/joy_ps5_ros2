from setuptools import find_packages, setup

package_name = 'joy_ps5_driver'

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
    maintainer='amb',
    maintainer_email='ambarishgk@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tester = joy_ps5_driver.tester:main',
            'touchpad = joy_ps5_driver.touchpad_publisher:main',
            'motion = joy_ps5_driver.motion_publisher:main',
            'combined = joy_ps5_driver.combined_publisher:main',
        ],
    },
)
