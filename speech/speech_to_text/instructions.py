import os

import requests

from misc_functions import strip_leading_space
import utils.names as namess
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

def get_loc():
    print("get_loc")
    req = requests.get("http://localhost:5000/getRelLoc")
    print(req.json())
    requests.get(f"http://localhost:5001/say/{req.json()['text']}")
class InstructionParser:
    instructions = {
        "print something": lambda: print("Printed Something"),
        "what is your name": lambda: print("My name is Howard!"),
        "where am i" : get_loc
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
            requests.get(f"http://localhost:5000/go/{strip_dets(location)}")
            req = requests.get(f"http://localhost:5000/getLandmark/{strip_dets(location)}")
            resp: dict = req.json()
            print(resp)
            keys = resp.keys()
            if 'error' in keys:
                os.system("mpg321 ../resources/snippets/error.mp3")
            elif 'check' in keys:
                os.system("mpg321 ../resources/snippets/check.mp3")
                requests.get(f"http://localhost:5001/say/{resp['check']}")
            else:
                os.system('pwd')
                os.system("mpg321 ../resources/snippets/found.mp3")
                requests.get(f"http://localhost:5001/say/{str(resp['x'])}, {str(resp['y'])}")
                # interface with motion code
            return True
            # send to landmarks API
        elif instruction.__contains__("find"):
            name - strip_leading_space(instruction.split("find")[1])
            print(f"Finding {names.get_id(name)}")
            requests.get(f"http://localhost:5000/say/Finding {names.get_id(name)}")
            requests.get(f"http://localhost:5000/seek/{names.get_id(name)}")

        else:
            try:
                self.instructions[instruction]()
                return True
            except KeyError:
                return False
