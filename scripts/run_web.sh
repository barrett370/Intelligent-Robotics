#!/bin/bash
sleep $1
cd $2/src
export FLASK_APP=webInterface/app.py
export FLASK_RUN_PORT=4200
flask run --host=0.0.0.0
