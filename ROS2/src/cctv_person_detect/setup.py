from setuptools import setup

package_name = 'cctv_person_detect'

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
            'coordinate_publisher = cctv_person_detect.coordinate_publisher:main',
            'coordinate_publisher_doingnot = cctv_person_detect.coordinate_publisher_doingnot:main',
            'coordinate_subscriber = cctv_person_detect.coordinate_subscriber:main',
            'test = cctv_person_detect.test:main',
            'webcam = cctv_person_detect.webcam:main
        ],
    },
)
