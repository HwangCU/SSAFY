from setuptools import find_packages, setup

package_name = 'rs_camera'

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
    maintainer='edurobot',
    maintainer_email='edurobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = rs_camera.image_subscriber:main',
            'depth_subscriber = rs_camera.depth_subscriber:main',
            'resize_data = rs_camera.resize_data:main',
             'image_crop = rs_camera.image_crop:main',
        ],
    },
)
