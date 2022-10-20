Command Line Tools
===================================

The ROSData Python Package comes with several command line tools to help perform common tasks such as extraction and visualization. 

Usage
-------------

The ROSData command line tools can be called by running the following command after installation:

.. code-block:: bash

    rosdata <tool> <tool_arguments>

To run the extraction tool for example, one would run the following:

.. code-block:: bash

    rosdata extract <extract_arguments>

Type in :code:`-h` or :code:`--help` to get a list of the available tools or the arguments for a specific tool. For example,

.. code-block:: bash

    rosdata --help          # to show the list of available tools
    rosdata <tool> --help   # to show the arguments for a specific tool


Example
-----------------

We have provided a complete extraction example. Download the :download:`example (50MB) <../assets/example.zip>` and extract it to any location. Then run the following commands:

.. code-block:: bash

    cd <path_to_extracted_example>
    rosdata extract example.bag example_extraction_config.yaml data

This example is extracting the data contained within the :code:`example.bag` using the specifications contained within the :code:`example_extraction_config.yaml` and outputting the extracted data into a folder located at :code:`<path_to_extracted_example>/data`. 