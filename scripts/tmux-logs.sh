#!/bin/bash

base_dir='$HOME/.ros/logs/latest'

tmux new-session -d 'tail -f $HOME/.ros/logs/latest/landmarks-5-stdout.log'
tmux split-window -v 'tail -f $HOME/.ros/logs/latest/tts-7-stdout.log'
tmux split-window -h 'tail -f $HOME/.ros/logs/latest/web-6-stdout.log'
tmux -2 attach-session -d

