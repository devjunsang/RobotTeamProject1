from setuptools import setup

pkg = 'aris3_gesture'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', ['launch/gesture.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'gesture_node = aris3_gesture.gesture_node:main',
    ]},
)
