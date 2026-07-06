from setuptools import find_packages, setup

package_name = 'panorama_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        # package_name: ['gazebo_room_coco.onnx'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dcx',
    maintainer_email='greatrun2023@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'display_four_camera = panorama_camera.display_four_camera:main',
        ],
    },
)
