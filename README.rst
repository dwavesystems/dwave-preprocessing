.. image:: https://circleci.com/gh/dwavesystems/dwave-preprocessing.svg?style=svg
    :target: https://circleci.com/gh/dwavesystems/dwave-preprocessing
    :alt: Linux/Mac/Windows build status

.. image:: https://codecov.io/gh/dwavesystems/dwave-preprocessing/branch/main/graph/badge.svg
    :target: https://codecov.io/gh/dwavesystems/dwave-preprocessing
    :alt: Code coverage

.. image:: https://badge.fury.io/py/dwave-preprocessing.svg
    :target: https://badge.fury.io/py/dwave-preprocessing
    :alt: Last version on PyPI

.. image:: https://img.shields.io/pypi/pyversions/dwave-preprocessing.svg?style=flat
    :target: https://pypi.org/project/dwave-preprocessing/
    :alt: PyPI - Python Version


===================
dwave-preprocessing
===================

.. start_preprocessing_about

`dwave-preprocessing` provides preprocessing tools for binary quadratic models
(BQM) and presolve algorithms for constrained quadratic models (CQM).

This package contains several preprocessing composites that can aid in solving
BQMs and a presolver that can reduce a problem's redundant variables and
constraints to improve the accuracy of CQMs. For details on underlying
algorithms and usage, see the
`documentation <https://docs.dwavequantum.com/en/latest/index.html>`_.

.. end_preprocessing_about

Installation
============

Install from a package on PyPI:

.. code-block:: bash

    pip install dwave-preprocessing

or install from source:

.. code-block:: bash

    pip install -r requirements.txt
    python setup.py build_ext --inplace
    python setup.py install

License
=======

Released under the Apache License 2.0. See `<LICENSE>`_ file.

Contributing
============

Ocean's `contributing guide <https://docs.dwavequantum.com/en/latest/ocean/contribute.html>`_
has guidelines for contributing to Ocean packages.

Release Notes
-------------

**dwave-preprocessing** makes use of `reno <https://docs.openstack.org/reno/>`_
to manage its release notes.

When making a contribution to **dwave-preprocessing** that will affect users,
create a new release note file by running

.. code-block:: bash

    reno new your-short-descriptor-here

You can then edit the file created under ``releasenotes/notes/``.
Remove any sections not relevant to your changes.
Commit the file along with your changes.
