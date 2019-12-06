#!/bin/bash
sleep $1
export GOOGLE_APPLICATION_CREDENTIALS=$2/creds.json
export FLASK_APP=landmarks/landmark.py

cd $2/src
flask run