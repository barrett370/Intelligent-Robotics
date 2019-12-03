#!/bin/bash

BLUE='\033[0;34m'
NC='\033[0m'

key="$1"
case $key in
    landmarks)
    echo -e "${BLUE} launching landmark server${NC}"
    pipenv run python ./landmarks/landmark/landmark.py
    ;;
    t2sp)
    echo -e "${BLUE} launching text_to_speech server${NC}"
    pipenv run python ./speech/text_to_speech/server/t2sp_server.py
    ;;
    sp2t)
    pwd
    echo -e "${BLUE} starting audio instruction parsing${NC}"
    pipenv run python ./speech/speech_to_text/gspeech_live.py
    ;;
esac
