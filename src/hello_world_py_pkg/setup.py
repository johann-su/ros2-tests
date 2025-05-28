from setuptools import find_packages, setup

package_name = 'hello_world_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='johann123.js@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello_world_node = hello_world_py_pkg.hello_world_node:main", # node_name = path.to.file:main_function -> ros2 run pkg_name node_name
            "custom_node = hello_world_py_pkg.custom_node:main",
            "simple_pub = hello_world_py_pkg.simple_pub:main",
            "simple_sub = hello_world_py_pkg.simple_sub:main",
            "simple_client = hello_world_py_pkg.simple_client:main",
            "simple_srv = hello_world_py_pkg.simple_srv:main",
            "simple_params = hello_world_py_pkg.simple_params:main",
        ],
    },
)
