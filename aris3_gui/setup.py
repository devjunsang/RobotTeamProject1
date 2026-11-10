from setuptools import setup

pkg = 'aris3_gui'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg, pkg + '.panels'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'control_station = aris3_gui.app:main',
    ]},
)
