from setuptools import setup

package_name = 'nif_imitative_planning_nodes'

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
    maintainer='cyJung',
    maintainer_email='cy.jung@kaist.ac.kr',
    description='ROS2 Package for imitative planning using PyTorch',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'imitative_planner = nif_imitative_planning_nodes.planning_node:main',
        ],
    },
)
