from setuptools import setup

package_name = 'depth_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hdw',
    maintainer_email='hdwook3918@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'webcam_publisher = depth_package.webcam_publisher:main',
            # 'webcam_subscriber = depth_package.webcam_subscriber:main',
            'realsense_publisher = depth_package.realsense_publisher:main',
            # 'realsense_subscriber = depth_package.realsense_subscriber:main',
            'center_publisher = depth_package.center_publisher:main',
            'depth_subscriber = depth_package.depth_subscriber:main'
        ],
    },
)
