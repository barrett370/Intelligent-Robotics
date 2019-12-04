#!/bin/bash
sleep 4
cd $1
export FLASK_APP=webInterface/app.py
export FLASK_RUN_PORT=4200
flask run --host=0.0.0.0
