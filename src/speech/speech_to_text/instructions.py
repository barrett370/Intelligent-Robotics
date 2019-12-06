import os

import requests
import difflib
import datetime

from speech.speech_to_text.misc_functions import strip_leading_space

from speech.speech_to_text.names import get_id

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


def get_weather():
    resp = requests.get("http://www.wttr.in?format=j1").json()['current_condition'][0]
    response = f"It is currently {resp['weatherDesc'][0]['value']} and feels like {resp['FeelsLikeC']} degrees"
    print(response)
    requests.get(f"http://localhost:5001/say/{response}")


def get_time():
    response = f"It is {datetime.datetime.now().hour} {datetime.datetime.now().minute}"
    print(response)
    requests.get(f"http://localhost:5001/say/{response}")


class InstructionParser:

    def __init__(self):
        self.prev_instruction = ""

    instructions = {
        "print something": lambda: print("Printed Something"),
        "what is your name": lambda: print("My name is Howard!"),
        "where am i": lambda: get_loc(),
        "what is the weather": lambda: get_weather(),
        "what is the time": lambda: get_time(),
        "what time is it": lambda: get_time(),
    }

    def parse(self, instruction: str, continued: bool) -> bool:
        """         

        :rtype: boolean of whether an instruction has been parsed & executed, allowing script to listen for wake word only
        """
        instruction = expand_abbreviations(instruction)
        if not continued:
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
                name = strip_leading_space(instruction.split("find")[1])
                print(f"Finding {get_id(name)}")
                requests.get(f"http://localhost:5001/say/Finding {get_id(name)}")
                requests.get(f"http://localhost:5000/seek/{name}")
            elif instruction.__contains__("learn my face"):
                response = "What is your name?"
                requests.get(f"http://localhost:5001/say/{response}")
                self.prev_instruction = 'learn my face'
            else:
                try:
                    max_sim = 0
                    max_instruction = ""
                    for poss_instruction in list(self.instructions.keys()):
                        print(poss_instruction)
                        sim = difflib.SequenceMatcher(a=instruction.lower(), b=poss_instruction.lower()).ratio()
                        if sim > max_sim:
                            print("updating")
                            max_sim = sim
                            max_instruction = poss_instruction
                    print(f"Max sim: {max_sim}, most similar instruction: {max_instruction}")
                    if max_sim > 0.7:
                        self.instructions[max_instruction]()
                        return True
                    else:
                        return True
                except KeyError:
                    return True
        else:
            print(f"continuing conversation from {self.prev_instruction}")
            if self.prev_instruction == "learn my face":
                if instruction.__contains__("my name is"):
                    name = strip_leading_space(instruction.split("my name is")[1])
                else:
                    name = instruction
                response = f"Learning face for {name}"
                print(response)
                resp = requests.get(f"http://localhost:5000/learn/{name}")
                print(resp.status_code)
                if resp.json()['text'] == "success":
                    response = "Learnt your face"
                    requests.get(f"http://localhost:5001/say/{response}")
                    print(response)
                else:
                    response = "Sorry something went wrong"
                    requests.get(f"http://localhost:5001/say/{response}")
                    print(response)
