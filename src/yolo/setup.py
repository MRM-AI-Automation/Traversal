from setuptools import setup

package_name = 'yolo'

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
    maintainer='mrm',
    maintainer_email='mrm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference=yolo.inference:main',
            'inference_depth=yolo.inference_depth:main',
            'inference_depth_simulation=yolo.inference_depth_simulation:main',
            'inference_engine=yolo.inference_engine:main',
            'imagestream=yolo.imagestream:main',
        ],
    },
)
