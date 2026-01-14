from setuptools import find_packages
from setuptools import setup

setup(
    name='dss_lio_sam',
    version='1.0.0',
    packages=find_packages(
        include=('dss_lio_sam', 'dss_lio_sam.*')),
)
