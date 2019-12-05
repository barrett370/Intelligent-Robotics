#!/bin/bash
sleep $2
cd $2/src/speech/speech_to_text/
python gspeech_live.py
