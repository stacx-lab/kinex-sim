from glob import glob

from setuptools import find_packages, setup

setup(
    name="kinex_sim_runtime", version="0.2.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/kinex_sim_runtime"]),
        ("share/kinex_sim_runtime", ["package.xml"]),
        ("share/kinex_sim_runtime/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "kinex-sim"],
    entry_points={"console_scripts": ["sim_node = kinex_sim_runtime.sim_node:main"]},
)
