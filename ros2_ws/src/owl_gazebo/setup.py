from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'owl_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'world'),
         glob('world/*.world')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.sdf')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.config')),
    ],
    install_requires=['setuptools', 'owl_description', 'owl_moveit_config'],
    zip_safe=True,
    maintainer='vardan',
    maintainer_email='vardanmittal000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
