from setuptools import setup

setup(
    name='dynamixel_gripper_ros2',
    version='0.0.0',
    packages=[],
    py_modules=['dynamixel_gripper'],
    install_requires=['setuptools'],
    author='Poh Yong Keat',
    author_email='yongkeat.poh@hopetechinik.com',
    maintainer='Poh Yong Keat',
    maintainer_email='yongkeat.poh@hopetechinik.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing examples of how to use the DYNAMIXEL ROS 2 API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'dynamixel_gripper = dynamixel_gripper:main'
        ],
    },
)
