from setuptools import setup

package_name = 'wheelchair_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katsuya',
    maintainer_email='ba2vwbus13wind@gmail.com',
    description='2D LiDARで人を検出して追従する電動車椅子の制御ノード',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower = wheelchair_follower.follower_node:main',
            'camera_follower = wheelchair_follower.camera_follower_node:main',
            'yolo_follower = wheelchair_follower.yolo_follower_node:main',
            'nav_follower = wheelchair_follower.nav_follower_node:main',
            'person_dancer = wheelchair_follower.person_dancer_node:main',
            'dance_mimic = wheelchair_follower.dance_mimic_node:main',
            'person_mover = wheelchair_follower.person_mover:main',
        ],
    },
)
