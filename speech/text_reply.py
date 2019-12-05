# /usr/bin/env python
# Download the twilio-python library from twilio.com/docs/libraries/python
import speech.text_sender
from flask import Flask, request
from twilio.twiml.messaging_response import MessagingResponse

app = Flask(__name__)


@app.route("/sms", methods=['GET', 'POST'])
def sms_ahoy_reply():
    """Respond to incoming messages with a friendly SMS."""
    # Start our response
    resp = MessagingResponse()
    message_contents = request.form['Body']
    recipient = request.form["From"]

    # Add a message
    if message_contents.upper() == "HELLO":
        resp.message("Welcome! What is your name? " + str(recipient))
    else:
        resp.message("Go away, " + str(recipient))
    return str(resp)


if __name__ == "__main__":
    app.run(debug=True)