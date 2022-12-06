#!/bin/sh
export PYTHONPATH=$PWD:$PWD/serialMaster
python3 opencat_imitation/imitation.py -v0
