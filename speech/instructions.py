import os

import requests

from speech.utils import strip_leading_space
from gtts import gTTS

language = 'en'


# experimental
def expand_abbreviations(instruction: str) -> str:
    if instruction.__contains__("'s"):
        instruction = instruction.replace("'s", " is")

    return instruction


def strip_dets(instruction: str) -> str:
    dets = ["the", "a", "an"]
    for det in dets:
        instruction = instruction.replace(det, "")
    return instruction


class InstructionParser:
    instructions = {
        "print something": lambda: print("Printed Something"),
        "what is your name": lambda: print("My name is Howard!")
    }

    def parse(self, instruction: str) -> bool:
        """

        :rtype: boolean of whether an instruction has been parsed & executed, allowing script to listen for wake word only
        """
        instruction = expand_abbreviations(instruction)
        print(f"expanded instruction: {instruction} ")
        if instruction.__contains__("take me to"):
            location = strip_leading_space(instruction.split("take me to")[1])
            print(f"You want to be taken to {location}")
            # Check if a valid location
            req = requests.get(f"http://localhost:5000/getLandmark/{strip_dets(location)}")
            resp: dict = req.json()
            print(resp)
            keys = resp.keys()
            if 'error' in keys:
                os.system("mpg321 ./resources/snippets/error.mp3")
            elif 'check' in keys:
                os.system("mpg321 ./resources/snippets/check.mp3")
                requests.get(f"http://localhost:5001/say/{resp['check']}")
            else:
                os.system("mpg321 ./resources/snippets/found.mp3")
                requests.get(f"http://localhost:5001/say/{str(resp['x'])}, {str(resp['y'])}")
                # interface with motion code
            return True
            # send to landmarks API
        else:
            try:
                self.instructions[instruction]()
                return True
            except KeyError:
                return False
