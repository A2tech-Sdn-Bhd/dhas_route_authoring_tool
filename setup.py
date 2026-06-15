from glob import glob
from setuptools import setup

package_name = 'route_authoring_tool'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leggeed',
    maintainer_email='leafloat.com@gmail.com',
    description='Offline route authoring tool for hybrid_smooth_path_follower.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_route = route_authoring_tool.cli:bag_to_route_main',
            'route_editor = route_authoring_tool.cli:route_editor_main',
        ],
    },
)
