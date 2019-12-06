import os
import playsound
import logging
from flask import Flask
from gtts import gTTS

app = Flask(__name__)
logging.getLogger('wekzeug').setLevel(logging.ERROR)

@app.route("/healthCheck")
def healthcheck() -> str:
    return "Text to Speech server is running"


@app.route("/say/<string>")
def say(string: str) -> str:
    language = 'en-GB'
    snippet = gTTS(text=string, lang=language, slow=False)
    # snippet.save(f"../resources/snippets/{key}.mp3")
    snippet.save("../../resources/tmp.mp3")
    os.system("mpg321 ../../resources/tmp.mp3")
    return "success"


if __name__ == "__main__":
    app.run(port=5001)
