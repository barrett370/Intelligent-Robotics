from speech.utils import strip_leading_space
# experimental
def expand_abbreviations(instruction: str) -> str:
    if instruction.__contains__("'s"):
        instruction = instruction.replace("'s", " is")

    return instruction


class InstructionParser:
    instructions = {
        "print something": lambda: print("Printed Something"),
        "what is your name": lambda: print("My name is Howard!")
    }

    def parse(self, instruction: str) -> bool:
        instruction = expand_abbreviations(instruction)
        print(f"expanded instruction: {instruction} ")
        if instruction.__contains__("take me to"):
            location = strip_leading_space(instruction.split("take me to")[1])
            print(f"You want to be taken to {location}")
            return True
            # send to landmarks API
        else:
            try:
                self.instructions[instruction]()
                return True
            except KeyError:
                return False
