Create a stereo agent
#####################

:summary: This example shows how to create an agent with a stereo camera pair.

This can be done by giving the agent two sensors (be it RGB, depth, or
semantic) with different positions. Note that the cameras must have different
UUIDs.

Example is runnable via

.. code:: shell-session

    $ python examples/tutorials/stereo_agent.py

.. include:: ../../examples/tutorials/stereo_agent.py
    :code: py
    :class: m-console-wrap
    :start-line: 5
