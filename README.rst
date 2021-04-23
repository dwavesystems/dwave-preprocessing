.. image:: https://circleci.com/gh/dwavesystems/dwave-preprocessing.svg?style=svg
    :target: https://circleci.com/gh/dwavesystems/dwave-preprocessing
    :alt: Linux/Mac/Windows build status

.. image:: https://codecov.io/gh/dwavesystems/dwave-preprocessing/branch/master/graph/badge.svg?token=ZkZo09uAl7
    :target: https://codecov.io/gh/dwavesystems/dwave-preprocessing
    :alt: Code coverage

.. image:: https://readthedocs.com/projects/d-wave-systems-dwave-preprocessing/badge/?version=latest
    :target: https://docs.ocean.dwavesys.com/projects/preprocessing/en/latest/
    :alt: Documentation status

.. image:: https://badge.fury.io/py/dwave-preprocessing.svg
    :target: https://badge.fury.io/py/dwave-preprocessing
    :alt: Last version on PyPI

.. image:: https://img.shields.io/pypi/pyversions/dwave-preprocessing.svg?style=flat
    :target: https://pypi.org/project/dwave-preprocessing/
    :alt: PyPI - Python Version


===================
dwave-preprocessing
===================

.. index-start-marker

`dwave-preprocessing` is a package of common preprocessing tools that may be used
for solving binary quadratic models (BQM).

.. code-block:: python

    import dwave.preprocessing

Currently, this package contains an implementation of roof duality, an algorithm 
used for finding minimizing assignments of a polynomial's variables. For details 
on the algorithm and how to use it, see the package's Reference Documentation.

.. index-end-marker

Installation
============

.. installation-start-marker

Install from a package on PyPI:

.. code-block:: bash

    pip install dwave-preprocessing

or install from source:

.. code-block:: bash

    USE_CYTHON=1 pip install git+https://github.com/dwavesystems/dwave-preprocessing.git#egg=dwave-preprocessing

Note: ``USE_CYTHON=1`` forces Cythonization and proper build from source. When
building from *PyPI package* source (which includes Cythonized files), this is
not necessary.

To build from source:

.. code-block:: bash

    pip install -r requirements.txt
    python setup.py build_ext --inplace
    python setup.py install

.. installation-end-marker

License
=======

Released under the Apache License 2.0. See `<LICENSE>`_ file.
