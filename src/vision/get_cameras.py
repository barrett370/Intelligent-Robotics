import glob

def get():
    counter = 0
    devs = []
    for dev in glob.glob("/dev/video*"):
        devs.append({"name": dev,"id": counter,"active": False})
        counter += 1
    return devs 