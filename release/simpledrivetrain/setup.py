import setuptools

with open("README.md", "r") as fh:
    long_desc = fh.read()

setuptools.setup(
    name="simpledrivetrain",
    version="1.0.1",
    author="Cordell Michaud",
    author_email="michaudcordell@users.noreply.github.com",
    description="A Python library that facilitates the control of robot drivetrains with complex motor arrangements.",
    long_description=long_desc,
    long_description_content_type="text/markdown",
    url="https://github.com/michaudcordell/SimpleDrivetrain",
    packages=setuptools.find_packages(),
    classifiers=(
        "Development Status :: 5 - Production/Stable",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "Intended Audience :: Science/Research",
    ),
)
