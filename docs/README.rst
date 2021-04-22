.. image:: https://circleci.com/gh/dwavesystems/dwave-preprocessing.svg?style=svg
    :target: https://circleci.com/gh/dwavesystems/dwave-preprocessing
    :alt: Linux/Mac build status

.. image:: https://ci.appveyor.com/api/projects/status/hcp8pxgdvbl0qimi/branch/master?svg=true
    :target: https://ci.appveyor.com/project/dwave-adtt/dwave-preprocessing/branch/master
    :alt: Windows build status

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

A package containing common preprocessing tools that may be useful when solving
binary quadratic models.

Currently, `dwave-preprocessing` contains an implementation of the roof_duality
algorithm, which may be accessed through `fixed_variables()` or the `RoofDualityComposite`.

.. code-block:: python

    >>> import dwave.preprocessing

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
