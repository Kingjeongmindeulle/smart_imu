from setuptools import find_packages, setup
import os               #추가 해줘야 하는 부분, 그 비로 아래 한칸도 무조건
from glob import glob

package_name = 'smart_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),  #추가

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimjeongmin',
    maintainer_email='kimjeongmin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'accelation_ro_pi = smart_imu.accelation_ro_pi:main',
            'angular_accelation_ya_ro_pi = smart_imu.angular_accelation_ya_ro_pi:main',
            'imu_pub = smart_imu.imu_pub:main',
            'magnetic_field_ya = smart_imu.magnetic_field_ya:main',
            'pos_vel = smart_imu.pos_vel:main',
        ],
    },
)
