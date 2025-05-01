from setuptools import find_packages, setup

package_name = 'track_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='suchitha.channa@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_converter = track_segmentation.extract_mp4:main',
            'track_follower = track_segmentation.track_path:main'
        ],
    },
)
