from setuptools import find_packages
from setuptools import setup

setup(
    name='livox_lio_sam',
    version='1.0.0',
    packages=find_packages(
        include=('livox_lio_sam', 'livox_lio_sam.*')),
)
