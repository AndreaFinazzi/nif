from setuptools import setup

package_name = 'nifpy_common_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_dir={'': package_name},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usrg',
    maintainer_email='finazzi@kaist.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
