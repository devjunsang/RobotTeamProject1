from setuptools import setup

pkg = 'aris3_sim_bridge'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'vision_bridge = aris3_sim_bridge.vision_bridge:main',
        'gripper_bridge = aris3_sim_bridge.gripper_bridge:main',
        'gz_gripper_bridge = aris3_sim_bridge.gz_gripper_bridge:main',
        'gz_truth_detector = aris3_sim_bridge.gz_truth_detector:main',
    ]},
)
