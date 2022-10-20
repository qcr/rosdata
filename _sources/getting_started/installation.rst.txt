Installation
=============

Install via Conda
------------------

Conda package to come.

Install via Pip
----------------

Pip package to come.


Install from Source
--------------------

We recommend using a conda virtual environment when installing from source to ensure the ROSData module dependencies do not conflict with your other projects. We recommend the following procedure for installing the rosdata tool from source:

.. code-block:: bash
    :linenos:

    git clone https://github.com/qcr/rosdata
    cd <rosdata_repo>
    conda env create --file rosdata_env.yml
    conda activate rosdata_env
    pip install -e .

The rosdata tool can then be called from any directory using the command :code:`rosdata <tool> <tool_arguments>`. If you do not wish to use a conda virtual environment, replace lines 3 and 4 from the code above with commands appropriate to your setup.