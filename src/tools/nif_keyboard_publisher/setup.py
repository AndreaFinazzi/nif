from setuptools import setup

package_name = 'nif_keyboard_publisher'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    install_requires=['setuptools'],
    description='Simple Publisher on Python',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher.py = nif_keyboard_publisher.publisher:main'
        ],
    },
)