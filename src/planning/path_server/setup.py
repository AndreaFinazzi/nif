from setuptools import setup
import os
from glob import glob

package_name = 'path_server'
package_dir = dir_path = os.path.dirname(os.path.realpath(__file__))

def copy_dir(dir):
    base_dir = os.path.join(package_dir,  dir)

    for (dirpath, dirnames, files) in os.walk(base_dir):
        for f in files:
            if not ('.git' in dirpath):
                full_path   = os.path.join(dirpath, f).replace(package_dir, '').split('/', 1)[1]
                dest        = dirpath.replace(package_dir, '').split('/', 1)[1]
                yield dest, full_path

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
data_files.extend(
    [ (os.path.join('share/' + package_name, d), [f]) for d, f in copy_dir('config')]
)
data_files.extend(
    [ (os.path.join('share/' + package_name, d), [f]) for d, f in copy_dir('maps')]
)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matt Boler',
    maintainer_email='meb0054@auburn.edu',
    description='Path generator for basic GPS following',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_server = path_server.path_server:main'
        ],
    },
)
