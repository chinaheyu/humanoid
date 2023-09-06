from setuptools import setup
from glob import glob
import os

package_name = 'humanoid_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'frames'), glob(os.path.join('frames', '*.json')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='810130242@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_arm_node = humanoid_arm.humanoid_arm_node:main',
            'teach = humanoid_arm.teach:main',
        ],
    },
)
