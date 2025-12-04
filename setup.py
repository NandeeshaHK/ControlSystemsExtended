from setuptools import setup, find_packages

setup(
    name="control_systems_extended",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "pymavlink",
        "dronekit",
        "numpy",
        "pandas",
        "keyboard"
    ],
)
