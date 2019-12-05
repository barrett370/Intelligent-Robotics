# /usr/bin/env python
# Download the twilio-python library from twilio.com/docs/libraries/python
# import speech.text_sender
from flask import Flask, request
from twilio.rest import Client
from twilio.twiml.messaging_response import MessagingResponse
import json, requests

# to run, run this py file on console along with:
# twilio phone-numbers:update "+447480485629" --sms-url="http://localhost:5000/sms"

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

    if recipient in contact_dictionary:
        username = contact_dictionary[recipient]
        print("user found: ", username)
        keyword = message_contents.split(" ")[0].upper()  # sets keyword as case insensitive word before any whitespace
        if keyword == "FIND":
            print("helping : " + username)
            message = "on my way to help you, " + username
            requests.get(f"http://localhost:5000/seek/{username}")
        elif keyword == "LEARN":
            message = "Learning your face, " + username + ", stand still!"
            resp = requests.get(f"http://localhost:5000/learn/{username}")
        elif keyword == "HOWARD":
            message = "Current commands are: 'Learn' and 'Find' (Case Insensitive)"
        else:
            message = "I don't know what you're after there buddy! For a list of commands, respond with 'Howard'"

    else:
        print(message_contents.split(":")[0].upper)
        if message_contents.split(":")[0].upper() == "REGISTER":
            given_name = message_contents.split(":")[1]
            print("new user")
            add_contact(given_name, recipient)
            message = "Welcome to Howard, the friendly guide bot, " + given_name
            print("adding: ", given_name, recipient)

        else:
            message = "you are not registered, please register with 'register:your_name_here"

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