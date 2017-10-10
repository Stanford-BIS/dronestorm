#!/bin/bash
# Script for running runtime_modules/run_control_attitude.py

TMUX_SESSION_NAME=run_control_attitude
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RUNDIR=${CURDIR}/../runtime_modules

tmux new-session -s ${TMUX_SESSION_NAME} -d "python ${RUNDIR}/comm/run_drone_comm.py"
tmux split -v "python ${RUNDIR}/comm/run_receiver.py"
tmux split -v "python ${RUNDIR}/control/run_control_attitude.py"
tmux select-layout -t ${TMUX_SESSION_NAME} even-vertical
tmux attach -t ${TMUX_SESSION_NAME}
