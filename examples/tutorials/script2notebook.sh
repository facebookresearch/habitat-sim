#!/bin/bash
jupytext --to notebook --update-metadata '{"accelerator": "GPU"}' *.py
