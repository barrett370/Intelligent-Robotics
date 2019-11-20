from gtts import gTTS
import os

text = "Hello, my name is Howard!"

language = 'en'

myobj = gTTS(text=text, lang=language, slow=False)

myobj.save("welcome.mp3")

os.system("mpg321 welcome.mp3")