#!/bin/bash
sleep $2
cd $2/src/speech/speech_to_text/
export GOOGLE_APPLICATION_CREDENTIALS=$2/creds.json
python gspeech_live.py
