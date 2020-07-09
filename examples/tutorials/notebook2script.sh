#!/bin/bash
jupytext --to py:percent --pipe black --pipe "sed s/[[:space:]]*\#[[:space:]]\%\%/\#\%\%/g" *.ipynb
