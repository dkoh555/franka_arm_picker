import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'diff_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'world'),
            glob(os.path.join('world', '*world*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        ('share/' + package_name + '/config/', ['config/ddrive.yaml']),
        # ('share/' + package_name, ['config/ddrive.yaml']),
        # ('share/' + package_name, ['config/ddrive_rviz.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damien',
    maintainer_email='damienkoh2025@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = diff_drive.test_node:main'
        ],
    },
)
