#!/bin/bash
# shellcheck disable=SC1090
source /home/rutgers_cait/.bashrc
nohup python /home/rutgers_cait/innov_uwb_api/localize_uwb.py > /dev/null 2>&1 &