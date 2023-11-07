from setuptools import find_packages, setup
import glob
import os

package_name = 'diff_drive'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name
                                                   ]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/env-hooks', ['env-hooks/ddrive.dsv']),
    ('share/' + package_name + '/worlds', glob.glob('worlds/*.world')),
    ('share/' + package_name + '/launch', glob.glob('launch/*launch.*')),
    ('share/' + package_name + '/config', glob.glob('config/*')),
    ('share/' + package_name + '/urdf', glob.glob('urdf/*.xacro')),
]


def get_data_files(package_name):
    for root, directories, files in os.walk("models"):
        for file in files:
            name = os.path.join(root, file)
            splitName = name.split(os.path.sep)
            folder = os.path.sep.join(
                name.split(os.path.sep)[:(len(splitName) - 1)])
            data_files.append(('share/' + package_name + "/" + folder, [name]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damien',
    maintainer_email='damienkoh2025@u.northwestern.edu',
    description='A Gazebo simulation for a flipping differential drive robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["flip = diff_drive.flip:flip_entry"],
    },
)