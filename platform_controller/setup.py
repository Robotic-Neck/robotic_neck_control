from setuptools import find_packages, setup

package_name = 'platform_controller'

data_files = []
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/platform_controller.launch.py']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='octa',
    maintainer_email='oeaguila@uc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_manager = platform_controller.motor_manager:main',
            'platform_manager = platform_controller.platform_manager:main'
        ],
    },
)
