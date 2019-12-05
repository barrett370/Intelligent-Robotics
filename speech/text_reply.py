# /usr/bin/env python
# Download the twilio-python library from twilio.com/docs/libraries/python
# import speech.text_sender
from flask import Flask, request
from twilio.rest import Client
from twilio.twiml.messaging_response import MessagingResponse
import json

app = Flask(__name__)
account_sid = 'ACeea156007a0ce498fdf1b5dc4c99bb80'#os.environ['TWILIO_ACCOUNT_SID']
auth_token = 'df00eacbd499e0c669a81cc91b8d0b5a'#os.environ['TWILIO_AUTH_TOKEN']
client = Client(account_sid, auth_token)

with open('addressBook.json', 'r') as json_file:
    contact_dictionary = json.load(json_file)


@app.route("/sms", methods=['GET', 'POST'])
def sms_ahoy_reply():
    """Respond to incoming messages with a friendly SMS."""
    # Start our response
    resp = MessagingResponse()
    message_contents = request.form['Body']
    recipient = request.form["From"]
    print(message_contents.upper())
    # Add a message
    if message_contents.split(":")[0] == "name":
        given_name = message_contents.split(":")[1]
        if recipient in contact_dictionary:
            print("recipient found")
            message = "Hello again, " + str(contact_dictionary[recipient])
            print(message)
        else:
            print("new user")
            add_contact(given_name, recipient)
            message = "new user"
            print("adding: ", given_name, recipient)
    else:
        message = "Go away, " + str(recipient)
        # send(message, recipient)
    write_json()
    print(message)
    resp.message(message)
    return str(resp)


def write_json():
    with open('addressBook.json', 'w') as json_file:
        json.dump(contact_dictionary, json_file)


def add_contact(name, number):
    contact_dictionary[number] = name


def send(message_body, recipient):
    message = client.messages \
                    .create(
                         body=message_body,
                         from_='+447480485629',
                         to=recipient
                     )

    print("message sent to : " + str(recipient))


if __name__ == "__main__":
    app.run(debug=True)