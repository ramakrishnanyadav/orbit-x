from setuptools import setup, find_packages

setup(
    name="orbit-x",
    version="1.0.0",
    description="ORBIT-X: Unified Aerospace Mission Planning Platform",
    author="ORBIT-X Development Team",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.9",
    install_requires=[
        "numpy>=1.24.0",
        "scipy>=1.10.0",
        "pandas>=2.0.0",
        "cvxpy>=1.3.0",
        "shapely>=2.0.0",
        "matplotlib>=3.7.0",
        "plotly>=5.14.0",
        "folium>=0.14.0",
        "astropy>=5.2.0",
        "skyfield>=1.45",
        "simplekml>=1.3.6",
        "tqdm>=4.65.0",
        "pyyaml>=6.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.3.0",
            "pytest-cov>=4.1.0",
        ]
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Physics",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
)
