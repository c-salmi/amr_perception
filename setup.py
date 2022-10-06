#!/usr/bin/env python3

import os
from setuptools import setup

directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(directory, 'README.md'), encoding='utf-8') as f:
  long_description = f.read()

setup(name='arm_perception',
      version='0.0.1',
      description='TUDelft AMR group perception pipeline',
      author='Chadi Salmi',
      license='MIT',
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages = ['amr_perception'],
      classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License"
      ],
      install_requires=[
          'easy_inference==0.0.3',
          'onnxruntime-gpu'
      ],
      python_requires='>=3.8',
      extras_require={},
      include_package_data=True)
