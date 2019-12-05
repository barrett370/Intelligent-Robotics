from gtts import gTTS
import os
import io
from playsound import playsound
language = 'en'
phrases = {
    'error': "Sorry, that location is not found",
    'check': "did you mean",
    'found': "Taking you to"
}

for key in phrases.keys():
    snippet = gTTS(text=phrases[key], lang=language, slow=False)
    #mp3_fp = BytesIO()
    #snippet.write_to_fp(mp3_fp)
    snippet.save(f"../resources/snippets/{key}.mp3")
    #playsound(f"../resources/snippets/{key}.mp3")
    os.system(f"mpg321 ../resources/snippets/{key}.mp3")
# os.system("mpg321 welcome.mp3")
