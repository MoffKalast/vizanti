from setuptools import setup

package_name = 'vizanti_demos'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MoffKalast',
    maintainer_email='vid.rijavec@gmail.com',
    description='Nav2 demo package for Vizanti',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_feeder = vizanti_demos.path_feeder:main'
        ],
    },
)
