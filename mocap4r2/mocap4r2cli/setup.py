from setuptools import find_packages
from setuptools import setup

package_name = 'mocap4r2cli'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Francisco Martín',
    author_email='fmrico@gmail.com',
    maintainer='Francisco Martín',
    maintainer_email='fmrico@gmail.com',
    url='https://github.com/MOCAP4ROS2-Project/mocap4r2/mocap4r2cli',
    download_url='https://github.com/MOCAP4ROS2-Project/mocap4r2/mocap4r2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Command line tools for MOCAP4ROS2 Project.',
    long_description="""\
The package provides the mocap4r2 command as a plugin for MOCAP4ROS2 Project.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'mocap4r2 = mocap4r2cli.command.mocap4r2:Mocap4r2Command',
        ],
        'ros2cli.extension_point': [
            'mocap4r2cli.verb = mocap4r2cli.verb:VerbExtension',
        ],
        'mocap4r2cli.verb': [
            'status = mocap4r2cli.verb.status:StatusVerb',
            'start = mocap4r2cli.verb.start:StartVerb',
            'stop = mocap4r2cli.verb.stop:StopVerb',
        ],
    }
)
