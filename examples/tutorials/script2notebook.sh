#!/bin/bash
jupytext --to notebook --update-metadata '{"jupytext": {"notebook_metadata_filter":"all"}, "accelerator": "GPU"}' --pipe black --pipe "sed s/[[:space:]]*\#[[:space:]]\%\%/\#\%\%/g" *.py
