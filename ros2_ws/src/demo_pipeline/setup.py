from setuptools import find_packages, setup

package_name = 'demo_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'demo_pipeline/semantic_model.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'synthetic_lidar = demo_pipeline.synthetic_lidar:main',
        'dataset_player = demo_pipeline.dataset_player:main',
        'ground_filter = demo_pipeline.ground_filter:main',
        'semantic_segmentation = demo_pipeline.semantic_segmentation:main',
    ]},
)