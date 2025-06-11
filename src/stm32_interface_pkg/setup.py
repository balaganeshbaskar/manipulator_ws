from setuptools import find_packages, setup

package_name = 'stm32_interface_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/uart.launch.py']),  # ✅ Add this line
],

    install_requires=['setuptools', 'pyserial'],  # ✅ pyserial added here
    zip_safe=True,
    maintainer='bgb_6342',
    maintainer_email='balaganeshbaskar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_uart_node = stm32_interface_pkg.stm32_uart_node:main',  # ✅ We'll create this script soon
        ],
    },
)
