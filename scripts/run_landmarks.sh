#!/bin/bash
sleep 4
export FLASK_APP=landmarks/landmark.py

cd $1
flask run