#!/bin/bash
sleep 4
export FLASK_APP=landmarks/landmark/landmark.py

cd $1
# cd landmarks/landmark
flask run