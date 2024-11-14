from setuptools import find_packages, setup

package_name = 'breakdown_function_handler'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='wangyongdong0817@gmail.com',
    description='This is a package for handling breakdown functions.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'breakdown_function_handler_node = breakdown_function_handler.breakdown_function_handler:main',
        ],
    },
)
