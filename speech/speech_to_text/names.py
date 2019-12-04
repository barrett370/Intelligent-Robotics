
import difflib
import requests

names = {
    "anant": 0 ,
    "charlie" : 1,
    "george" :2 ,
    "jon":3 , 
    "sam" : 4 
}


def get_id(name_input:str)->int:
    max_sim = 0
    sim_name = ""
    for name in list(names.keys()):
        sim = difflib.SequenceMatcher(a= name, b=name_input.lower()).ratio()
        if sim > max_sim:
            max_sim = sim
            sim_name = name     
    print(f"Max sim: {max_sim}, most similar name: {sim_name}")
    if max_sim > 0.5:
        return names[sim_name]
    else:
        string = "I don't know anyone by that name"
        requests.get(f"http://localhost:5001/say/{string}")
        return