#!/bin/bash
sleep $1
export GOOGLE_APPLICATION_CREDENTIALS=$2/creds.json
cd $2/src/speech/speech_to_text/
python gspeech_live.py
