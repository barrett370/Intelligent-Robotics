#!/bin/bash
sleep 4
cd $1
export FLASK_APP=speech/text_to_speech/server/t2sp_server.py
export FLASK_RUN_PORT=5001
flask run --host=0.0.0.0
