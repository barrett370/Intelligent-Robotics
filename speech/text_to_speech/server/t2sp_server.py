import io
import playsound

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
    mp3_fp = io.BytesIO()
    snippet.write_to_fp(mp3_fp)
    # snippet.save(f"../resources/snippets/{key}.mp3")
    playsound.playsound(mp3_fp)
    return "success"


if __name__ == "__main__":
    app.run(port=5001)
