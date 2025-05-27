from setuptools import setup, find_packages

setup(
    name='ros_node_interface',
    version='0.0.0',
    packages=['ros_node_interface'],
    package_dir={'': 'src'},
    install_requires=[
        'rospy',
        'threading',
    ],
)
