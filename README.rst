.. image:: https://circleci.com/gh/dwavesystems/dwave-preprocessing.svg?style=svg
    :target: https://circleci.com/gh/dwavesystems/dwave-preprocessing
    :alt: Linux/Mac/Windows build status

.. image:: https://codecov.io/gh/dwavesystems/dwave-preprocessing/branch/main/graph/badge.svg
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

`dwave-preprocessing` is a package of common preprocessing tools that can aid in 
solving binary quadratic models (BQM).

.. code-block:: python

    import dwave.preprocessing

Currently, this package contains several preprocessing composites. For details on
underlying algorithms and usage, see the package's 
`Reference Documentation <https://docs.ocean.dwavesys.com/en/stable/docs_preprocessing/reference/index.html>`_.

.. index-end-marker

Installation
============

.. installation-start-marker

Install from a package on PyPI:

.. code-block:: bash

    pip install dwave-preprocessing

or install from source:

.. code-block:: bash

    pip install -r requirements.txt
    python setup.py build_ext --inplace
    python setup.py install

.. installation-end-marker

License
=======

Released under the Apache License 2.0. See `<LICENSE>`_ file.
