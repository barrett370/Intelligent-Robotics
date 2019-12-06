from twilio.rest import Client

# Your Account Sid and Auth Token from twilio.com/console
# DANGER! This is insecure. See http://twil.io/secure
account_sid = 'ACeea156007a0ce498fdf1b5dc4c99bb80'#os.environ['TWILIO_ACCOUNT_SID']
auth_token = 'df00eacbd499e0c669a81cc91b8d0b5a'#os.environ['TWILIO_AUTH_TOKEN']
client = Client(account_sid, auth_token)


def send(message_body, recipient):
    message = client.messages \
                    .create(
                         body=message_body,
                         from_='+447480485629',
                         to=recipient
                     )

    print("message sent with sid : " + str(message.sid))


send("fuck u ", "+447804919284")
