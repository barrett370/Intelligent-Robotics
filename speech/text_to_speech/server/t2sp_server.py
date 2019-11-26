import os

from flask import Flask
from gtts import gTTS

app = Flask(__name__)


@app.route("/healthcheck")
def healthcheck() -> str:
    return "Text to Speech server is running"


@app.route("/say/<string>")
def say(string: str) -> str:
    language = 'en-GB'
    snippet = gTTS(text=string, lang=language, slow=False)
    os.system('pwd')
    snippet.save("../../resources/tmp.mp3")
    os.system("mpg321 ../../resources/tmp.mp3")
    return "success"


if __name__ == "__main__":
    app.run(port=5001)
