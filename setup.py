from glob import glob
import os

from setuptools import setup

package_name = "vizanti"

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
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "meshes"), glob("wiki_assets/*.png")),
    ] + generate_public_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoffKalast, Alastair Bradford",
    maintainer_email="vid.rijavec@gmail.com, albradford2468@gmail.com",
    description="A mission planner and visualizer for controlling outdoor ROS robots.",
    license="BSD3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vizanti_flask_node = vizanti.server:main",
            "vizanti_topic_handler_node = vizanti.topic_handler:main",
            "vizanti_service_handler_node = vizanti.service_handler:main",
        ],
    },
)
