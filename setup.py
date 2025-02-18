from setuptools import setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nithesh',
    maintainer_email='nithesh@example.com',
    description='Motor control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_publisher = motor_control.scripts.motor_publisher:main'
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ]
)

