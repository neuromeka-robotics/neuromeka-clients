from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()


setup(
    name="neuromeka-clients",
    version="0.1.4",
    author="Neuromeka",
    author_email="youngjin.heo@neuromeka.com",
    description="Neuromeka client protocols for Indy, IndyEye, Moby, Ecat, and Motor",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/neuromeka-robotics/neuromeka-clients",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    python_requires=">=3.6",
    install_requires=[
        "grpcio==1.39.0",
        "grpcio-tools==1.39.0",
        "protobuf==3.17.3",
        "requests",
        "Pillow",
        "numpy"
    ],
)