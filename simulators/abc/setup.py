from setuptools import find_packages, setup

package = "kinex_sim_abc"

setup(
    name=package,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package}"]),
        (f"share/{package}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "kinex_sim.backends": ["abc = kinex_sim_abc.backend:AbcSim"],
        "console_scripts": ["control_demo = kinex_sim_abc.demos.control:main"],
    },
)
