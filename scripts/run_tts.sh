#!/bin/bash
sleep 4
cd $1/speech/text_to_speech/server
export FLASK_APP=t2sp_server.py
export FLASK_RUN_PORT=5001
flask run --host=0.0.0.0