#!/bin/bash


tmux new-session -d 'tail -f $HOME/.ros/log/latest/landmarks-5-stdout.log'
tmux split-window -v 'tail -f $HOME/.ros/log/latest/tts-7-stdout.log'
tmux split-window -h 'tail -f $HOME/.ros/log/latest/rosout-1-stdout.log'
tmux select-pane -U
tmux split-window -h 'tail -f $HOME/.ros/log/latest/web-6-stdout.log'
tmux -2 attach-session -d

