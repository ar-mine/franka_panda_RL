from setuptools import setup

package_name = 'franka_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+".base"],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armine',
    maintainer_email='934218777@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'instance_detector = franka_perception.ObjDetectorNode:main',
            'box_detector = franka_perception.BoxDetectorNode:main',
            'move_box = franka_perception.MoveBoxNode:main'
        ],
    },
)
