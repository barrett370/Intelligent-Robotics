import requests

resp = requests.get("http://www.wttr.in?format=j1").json()['current_condition'][0]
print(resp)
response = f"It is currently {resp['weatherDesc'][0]['value']} and feels like {resp['FeelsLikeC']} degrees"
print(response)
requests.get(f"http://localhost:5001/say/{response}")