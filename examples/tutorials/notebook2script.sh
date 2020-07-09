#!/bin/bash
jupytext --to py:percent --update-metadata '{"jupytext": {"notebook_metadata_filter":"all"}, "accelerator": "GPU"}' --pipe black --pipe "sed s/[[:space:]]*\#[[:space:]]\%\%/\#\%\%/g" *.ipynb
