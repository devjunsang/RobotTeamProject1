from setuptools import setup

pkg = 'aris3_real_bridge'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', ['launch/real_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'real_bridge = aris3_real_bridge.bridge_node:main',
    ]},
)
