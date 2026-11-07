from glob import glob
from setuptools import setup

pkg = 'e0509_manual'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml', 'README.md']),
        ('share/' + pkg + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'manual_controller = e0509_manual.gui:main',
    ]},
)
