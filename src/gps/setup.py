from setuptools import setup

package_name = 'gps'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A package for parsing UBlox GPS data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_rtcm = gps.gps_rtcm:main',
            'gps_rtk= gps.gps_rtk:main',  # Entry point to your main function
            'odomframeidchanger= gps.odomframeidchanger:main', 
            'domainbridge=gps.domainbridge:main',
        ],
    },
    package_data={
        # Include the package.xml in the package installation
        package_name: ['package.xml'],
    },
    include_package_data=True,  # To include non-Python files specified in package_data
)
