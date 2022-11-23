#!/bin/bash
# shellcheck disable=SC1090
source ~/.bashrc
nohup python ~/innov_uwb_api/localize_uwb.py > /dev/null 2>&1 &