from setuptools import find_packages, setup
import os
from glob import glob

package_name = "gazebo_perception"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
]

for root, dirs, files in os.walk('models'):
    for file in files:
        data_files.append((os.path.join('share', package_name, root), [os.path.join(root, file)]))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,  
    install_requires=["setuptools"],
    maintainer="akshit",
    maintainer_email="akshitbhaskara@gmail.com",
    description="Streams organized point clouds from Gazebo to RViz2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = gazebo_perception.perception_node:main",
            "inference_node = gazebo_perception.yolo_inference_node:main",
            "perception_server = gazebo_perception.perception_server:main",
        ],
    },
)
