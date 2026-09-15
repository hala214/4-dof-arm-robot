from setuptools import setup, find_packages

setup(
    name="robot4dof-kit",
    version="0.1.0",
    description="Reusable 4-DOF robotic arm IK, serial controller, and GUI",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.23",
        "pyserial>=3.5",
    ],
    python_requires=">=3.9",
)
