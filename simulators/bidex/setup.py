from setuptools import find_packages, setup

package = "kinex_sim_bidex"

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
        "kinex_sim.backends": ["bidex = kinex_sim_bidex.backend:BiDexSim"],
        "console_scripts": ["control_demo = kinex_sim_bidex.demos.control:main"],
    },
)
