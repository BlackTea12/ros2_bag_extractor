from setuptools import find_packages, setup

package_name = 'ros2_bag_extractor'

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
    maintainer='Yaeohn Kim',
    maintainer_email='BlackTea12@github.com',
    description='ros2_bag_extractor package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = ros2_bag_extractor.main:main',
        ],
    },
)
