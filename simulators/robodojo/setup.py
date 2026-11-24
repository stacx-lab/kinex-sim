from setuptools import find_packages, setup

package = "kinex_sim_robodojo"

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
        "kinex_sim.backends": ["robodojo = kinex_sim_robodojo.worker:RoboDojoWorker"],
        "console_scripts": ["control_demo = kinex_sim_robodojo.demos.control:main"],
    },
)
