from setuptools import setup
import os
from glob import glob

package_name = 'nif_multilayer_planning_nodes'
package_dir = dir_path = os.path.dirname(os.path.realpath(__file__))


def copy_dir(dir_name):
    base_dir = os.path.join(package_dir, dir_name)

    for (dirpath, dirnames, files) in os.walk(base_dir):
        for f in files:
            if not ('.git' in dirpath):
                full_path = os.path.join(dirpath, f).replace(package_dir, '').split('/', 1)[1]
                dest = dirpath.replace(package_dir, '').split('/', 1)[1]
                yield dest, full_path


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*.launch.py')),
]
data_files.extend(
    [(d, [f]) for d, f in copy_dir('lib')]
)
data_files.extend(
    [(d, [f]) for d, f in copy_dir('assets')]
)
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usrg',
    maintainer_email='cy.jung@kaist.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nif_multilayer_planning_nodes_exe = nif_multilayer_planning_nodes.nif_multilayer_planning_node:main'
        ],
    },
)
