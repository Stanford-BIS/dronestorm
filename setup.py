#!/usr/bin/env python
import io

try:
    from setuptools import find_packages, setup
except ImportError:
    raise ImportError(
        "'setuptools' is required but not installed. To install it, "
        "follow the instructions at "
        "https://pip.pypa.io/en/stable/installing/#installing-with-get-pip-py")

def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)

setup(
    name='dronestorm',
    version='0.1',
    description='Python controller for a drone',
    long_description=read('README.md'),
    classifiers=[
      'Development Status :: 3 - Alpha',
      'License :: OSI Approved :: MIT License',
      'Programming Language :: Python :: 2.7',
      'Programming Language :: Python :: 3.5',
      'Intended Audience :: Science/Research'
    ],
    keywords='drone control',
    url='https://github.com/Stanford-BIS/dronestorm',
    author='Sam Fok, Arnav Gudibande',
    author_email='samfok@stanford.edu, arnav.gudibande@gmail.com',
    license='MIT',
    packages=find_packages(),
    install_requires=[
        'pyserial>=3.0',
        'pigpio>=1.35',
        'numpy>=1.7',
        'nengo>=2.4.0',
        'redis>=2.10.6',
    ],
    # test_suite='nose.collector',
    # tests_require=['nose', 'nose-cover3'],
    # entry_points={
    #     'console_scripts': ['funniest-joke=funniest.command_line:main'],
    # },
    include_package_data=True,
    zip_safe=False
)
