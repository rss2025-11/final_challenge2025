from setuptools import find_packages, setup
import os
import glob

package_name = 'final_challenge2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/final_challenge2025/launch/sim', glob.glob(os.path.join('launch', 'sim', '*launch.*'))),
        ('share/final_challenge2025/launch/real', glob.glob(os.path.join('launch', 'real', '*launch.*'))),
        # Include all config files
        (os.path.join('share', package_name, 'config', 'sim'), glob.glob('config/sim/*.yaml')),
        (os.path.join('share', package_name, 'config', 'real'), glob.glob('config/real/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='princepatel1304@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = final_challenge2025.state_machine:main',
            'pure_pursuit = final_challenge2025.pure_pursuit:main',
            'basement_point_publisher = final_challenge2025.basement_point_publisher:main',
            'detector = final_challenge2025.model.detection_node:main',
        ],
    },
)
