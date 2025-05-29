# Copyright 2021 D-Wave Systems Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext as _build_ext

import dimod

from Cython.Build import cythonize


extra_compile_args = {
    'msvc': ['/std:c++17', '/EHsc'],
    'unix': ['-std=c++17'],
}

extra_link_args = {
    'msvc': [],
    'unix': ['-std=c++17'],
}


class build_ext(_build_ext):
    def build_extensions(self):
        compiler = self.compiler.compiler_type

        compile_args = extra_compile_args[compiler]
        for ext in self.extensions:
            ext.extra_compile_args.extend(compile_args)

        link_args = extra_link_args[compiler]
        for ext in self.extensions:
            ext.extra_compile_args.extend(link_args)

        super().build_extensions()


setup(
    name='dwave-preprocessing',
    cmdclass=dict(build_ext=build_ext),
    ext_modules=cythonize(
        [Extension('dwave.preprocessing.cyfix_variables',
                   ['dwave/preprocessing/cyfix_variables.pyx']),
         Extension('dwave.preprocessing.presolve.cypresolve',
                   ['dwave/preprocessing/presolve/cypresolve.pyx']),
         ],
        annotate=True,
        nthreads=int(os.getenv('CYTHON_NTHREADS', 0)),
        ),
    include_dirs=[
        dimod.get_include(),
        'extern/spdlog/include/',
        ],
    install_requires=[
        'numpy>=1.17.3,<3.0.0',  # this is the oldest supported NumPy on Python 3.8
        'dimod>=0.12.20,<0.13.0'
        ],
)
