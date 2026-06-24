from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'final_countdown'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*')),),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sol',
    maintainer_email='a01747396@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'alineation_puzzle_ini=final_countdown.alineation_puzzle_ini:main',
            'alineation_puzzle=final_countdown.alineation_puzzle:main',
            'aruco_simu=final_countdown.aruco_simu:main',
            'baby_aruco=final_countdown.baby_aruco:main', 
            'bug0=final_countdown.bug0:main', 
            'localisation=final_countdown.localisation:main',
        ],
    },
)
