Logging Configuration
=====================

This page explains how to configure what is logged in Habitat-Sim.

Turn off non-critical logging
-----------------------------

To turn off non-critical logging, use one of the following based on your current version:

* Habitat-Sim version >= 0.2.2
  .. code-block::

    export MAGNUM_LOG=quiet HABITAT_SIM_LOG=quiet

* Habitat-Sim version < 0.2.2
  .. code-block::

    export MAGNUM_LOG=quiet GLOG_minloglevel=2

Turning on specific logging
---------------------------

Habitat-Sim has numerous different subsystems (gfx, physics, sim, scene, etc.) and these
can all have their logging levels set independently.  The ``HABITAT_SIM_LOG`` environment
variable takes a string that configures these which is governed by the following
grammar

.. code-block:: sh

    FilterString: SetLevelCommand (COLON SetLevelCommand)*
    SetLevelCommand: (SUBSYSTEM (COMMA SUBSYSTEM)* EQUALS)? LOGGING_LEVEL


where ``SUBSYSTEM`` is the name (or names) of a given subsystem and ``LOGGING_LEVEL`` is the logging level name. If no subsystem name is given, the level will be applied to all subsystems.

For example, ``HABITAT_SIM_LOG=quiet:physics,sim=verbose`` will set all subsystems to quiet and then set both the physics and sim subsystems to verbose.
Logging level names and subsystem names are not case sensitive in this filter configuration string.


Logging Level: :dox:`esp::logging::LoggingLevel`

Logging subsystems: :dox:`esp::logging::Subsystem`




GPU Context Debugging
---------------------

When trying to debug things related to GPU contexts, set the following

.. code-block:: sh

    export MAGNUM_LOG=verbose MAGNUM_GPU_VALIDATION=ON
