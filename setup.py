import os
from glob import glob
from setuptools import setup

package_name = 'r2b2'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        # (os.path.join('share', package_name, 'config', 'map'), glob('config/map/*')),
        # (os.path.join('share', package_name, 'config', 'params'), glob('config/params/*')),
        # (os.path.join('share', package_name, 'config', 'urdf'), glob('config/urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='sheaffej@gmail.com',
    description='The R2B2 robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'roboclaw_node = roboclaw_driver.roboclaw_node:main'
        ],
    },
)
