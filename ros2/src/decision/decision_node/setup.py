from setuptools import setup

package_name = 'decision_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nath',
    maintainer_email='nath@ejemplo.com',
    description='Nodo de decisión para robot móvil con evasión de obstáculos',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_node = decision_node.decision_node:main',
        ],
    },
)
