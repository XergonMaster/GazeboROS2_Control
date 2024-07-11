import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'racing_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
    'racing_bringup.ackerman_driver',
    'racing_bringup.Bicycle',  # Asegúrate de incluir el módulo Bicycle
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf_launch.py']),
        ('share/' + package_name + '/config', ['config/ekf_params.yaml']),  # Asegúrate de incluir el archivo de configuración
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi4master',
    maintainer_email='rpi4master@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackerman_driver = racing_bringup.ackerman_driver:main',

        ],
    },
)
