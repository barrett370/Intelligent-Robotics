#!/bin/bash
sleep $1
export FLASK_APP=landmarks/landmark.py

cd $2/src
flask run