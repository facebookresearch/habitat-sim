.. py:module:: habitat_sim.sensors.noise_models
    :summary: Library of Sensor noise models


    A library of noise models to close the gap between simulated observations
    and observations from real sensors.

    A noise model can be applied to a sensor by specifying the name of the noise
    model in the `sensor.SensorSpec.noise_model` feild.
    Arguments can be passed to the noise model constructor as keyword arguments using
    the `sensor.SensorSpec.noise_model_kwargs` field. For instance, to use the `RedwoodDepthNoiseModel`
    with a ``noise_multiplier`` of 5

    .. code:: py

        sensor_spec.noise_model = "RedwoodDepthNoiseModel"
        sensor_spec.noise_model_kwargs = dict(noise_multiplier=5)



    These noise models are commonly the result of contributions from various research projects.
    If you use a noise model in your research, please cite the relevant work specified by the docummentation


    **Depth Noise Models**

    * Redwood Noise Model for PrimSense depth cameras: `RedwoodDepthNoiseModel`

.. py:class:: habitat_sim.sensors.noise_models.NoSensorNoiseModel
    :summary: No noise noise model.  Simply returns a copy of the input

    Accessible from the registry under the name ``"None"``

.. py:class:: habitat_sim.sensors.noise_models.RedwoodDepthNoiseModel
    :summary: Redwood Noise Model for PrimSense depth cameras

    Accessible from the registry under the name ``"RedwoodDepthNoiseModel"``

    Implements the noise model provided by http://redwood-data.org/indoor/dataset.html

    If you use this noise model, please cite

    .. code:: bibtex

        @inproceedings{choi2015robust,
          title={Robust reconstruction of indoor scenes},
          author={Choi, Sungjoon and Zhou, Qian-Yi and Koltun, Vladlen},
          booktitle={Proceedings of the IEEE Conference on Computer Vision and
            Pattern Recognition}, pages={5556--5565}, year={2015}
        }


.. py:function:: habitat_sim.sensors.noise_models.RedwoodDepthNoiseModel.__init__
    :param gpu_device_id: The ID of CUDA device to use (only applicable if habitat-sim was built with ``--with-cuda``)
    :param noise_multiplier: Multipler for the Gaussian random-variables.  This reduces or increases the amount of noise
