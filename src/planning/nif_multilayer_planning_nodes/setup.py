from setuptools import setup
import os
from glob import glob

package_name = 'nif_multilayer_planning_nodes'

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
    maintainer='usrg',
    maintainer_email='cy.jung@kaist.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nif_mulitlayer_planner_node = nif_multilayer_planning_nodes.nif_multilayer_planning_node:main'
        ],
    },
)
