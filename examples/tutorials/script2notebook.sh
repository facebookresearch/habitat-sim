#!/bin/bash
jupytext --to notebook --update-metadata '{"accelerator": "GPU"}' --pipe black --pipe "sed s/[[:space:]]*\#[[:space:]]\%\%/\#\%\%/g" *.py
