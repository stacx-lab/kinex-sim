from setuptools import find_packages, setup

package = "kinex_sim_robocasa"

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
        "kinex_sim.backends": ["robocasa = kinex_sim_robocasa.backend:RoboCasaSim"],
        "console_scripts": ["control_demo = kinex_sim_robocasa.demos.control:main"],
    },
)
