from setuptools import find_packages, setup
import os

package_name = 'semantic_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), ['models/yolov8n.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkantastan',
    maintainer_email='btastan9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "yolo_detector = semantic_nav.yolo_node:main",
            "semantic_mapper = semantic_nav.semantic_mapper:main",
        ],
    },
)
