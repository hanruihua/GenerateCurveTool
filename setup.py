from setuptools import setup
import sys

setup(
    name='Generate curve tool',
    py_modules=['GCT'],
    version= '1.0',
    install_requires=[
        'matplotlib',
        'numpy',
        'scipy',
    ],
    description="A tool to generate the common curves for robot planning",
    author="Han Ruihua",
)