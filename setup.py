import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyro",
    version="0.6-dev",
    author="Alexandre Girard",
    author_email="alx87grd@gmail.com",
    description=("Toolbox for robot dynamic simulation, analysis, "
                 "control and planning"),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/SherbyRobotics/pyro",
    packages=['pyro'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering"
    ],
)
