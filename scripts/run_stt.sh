#!/bin/bash
sleep $2
cd $2/src/speech/speech_to_text/
export $2/GOOGLE_APPLICATION_CREDENTIALS=creds.json
python gspeech_live.py
