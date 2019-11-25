from gtts import gTTS
import os

language = 'en'
phrases = {
    'error': "Sorry, that location is not found",
    'check': "did you mean",
    'found': "Taking you to"
}

for key in phrases.keys():
    snippet = gTTS(text=phrases[key], lang=language, slow=False)
    snippet.save(f"../resources/snippets/{key}.mp3")

# os.system("mpg321 welcome.mp3")
