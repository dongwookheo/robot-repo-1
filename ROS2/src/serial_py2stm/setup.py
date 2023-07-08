from setuptools import setup

package_name = 'serial_py2stm'

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
            'serial_py2stm = serial_py2stm.serial_py2stm:main',
            # 'cmd_vel_serial_sender = serial_py2stm.cmd_vel_serial_sender:main'
        ],
    },
)
