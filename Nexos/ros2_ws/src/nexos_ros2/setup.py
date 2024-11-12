from setuptools import find_packages, setup

package_name = 'nexos_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/nexos.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Oliveros',
    maintainer_email='carlosalbertolivmo@gmail.com',
    description='Package for NexOS aplication',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nexos_app = nexos_ros2.nodo_app:main',
            'cinematica_inversa = nexos_ros2.nodo_cin_inv:main',
        ],
    },
)
