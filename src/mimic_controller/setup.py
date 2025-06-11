from setuptools import find_packages, setup

package_name = 'mimic_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/mimic_controller/launch', ['launch/mimic.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bgb_6342',
    maintainer_email='balaganeshbaskar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mimic_publisher = mimic_controller.mimic_input_publisher:main',
            'mimic_to_moveit = mimic_controller.mimic_to_moveit_action:main',
        ],  
    },
)
