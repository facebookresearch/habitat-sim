#!/bin/sh
#Synchronizes notebokos with script representations
pre-commit run 'jupytext' --files "$@"
