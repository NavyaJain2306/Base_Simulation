from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocup_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niharika',
    maintainer_email='niharikaprasad2019@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #'planner_node = robocup_planner.main:main',
            'task_planner = robocup_planner.task_planner_node:main',
            'workbench_planner = robocup_planner.workbench_planner_node:main',
            'task_executor = robocup_planner.task_executor_node:main',
            'temp_arm_executor = robocup_planner.temp_arm_executor_node:main',
            'temp_warehouse = robocup_planner.temp_warehouse_node:main',
        ],
    },
)
