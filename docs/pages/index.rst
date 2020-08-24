Habitat Simulator
#################

A flexible, high-performance 3D simulator with configurable agents, multiple
sensors, and generic 3D dataset handling (with built-in support for
MatterPort3D, Gibson, Replica, and other datasets).

When rendering a scene from the Matterport3D dataset, *Habitat-Sim* achieves
several thousand frames per second (FPS) running single-threaded, and reaches
over 10,000 FPS multi-process on a single GPU!

`Tutorials`_
============

.. TODO: this is waiting on m.css to propagate page titles to links

-   :ref:`New Actions <std:doc:new-actions>`
-   :ref:`Notebooks <std:doc:notebooks>`
-   :ref:`Creating a stereo agent <std:doc:stereo-agent>`
-   :ref:`Working with light setups <std:doc:lighting-setups>`
-   :ref:`Extracting Images <std:doc:image-extractor>`
-   :ref:`Interactive Rigid Objects <std:doc:rigid-object-tutorial>`

Checkout our `ECCV 2020 tutorial series`_ for quick-start code and video overviews of many core features!

.. _ECCV 2020 tutorial series: https://aihabitat.org/tutorial/2020/

.. block-info:: C++ API documentation

    Habitat-sim is designed to be used primarily through its Python API. As such, the
    end-user tutorials and docs linked above focus on Python.

    If you're looking for API reference of Habitat-sim's C++ internals, please see the
    `C++ API <cpp.html>`_ tab.
