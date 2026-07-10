import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'custom_grasp_pke'

# 获取launch目录下所有文件的绝对路径
launch_files = glob(os.path.join('launch', '*.py'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch目录下的所有文件
        (os.path.join('share', package_name, 'launch'), launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zeil',
    maintainer_email='2447234815@qq.com',
    description='ROS 2 integration nodes for vision-guided RM65 grasping.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_tf_node = custom_grasp_pke.vision_tf_node:main',
            'grasp_control_node = custom_grasp_pke.grasp_control_node:main',
        ],
    },
)
