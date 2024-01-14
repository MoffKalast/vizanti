import os
from glob import glob
from setuptools import setup

package_name = "vizanti_server"

def generate_public_data_files():
    data_files = []
    install_base = os.path.join("share", package_name)
    for root, dirs, files in os.walk("public"):
        install = os.path.join(install_base, root)
        sources = [os.path.join(root, f) for f in files]
        data_files.append((install, sources))
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ] + generate_public_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    author='MoffKalast',
    author_email='vid.rijavec@gmail.com',
    maintainer="MoffKalast, Alastair Bradford",
    maintainer_email="vid.rijavec@gmail.com, albradford2468@gmail.com",
    keywords=['vizanti', 'mission', 'planner'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD3',
        'Programming Language :: Python/Javascript',
        'Topic :: Visualization',
    ],
    description="A mission planner and visualizer for controlling outdoor ROS robots.",
    license="BSD3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vizanti_flask_node = vizanti_server.server:main",
            "vizanti_service_handler_node = vizanti_server.service_handler:main",
        ],
    },
)
