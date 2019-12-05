import pyaudio
import wave


audio = pyaudio.PyAudio()
def get():
    print("----------------------record device list---------------------")
    info = audio.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    devices = []
    for i in range(0, numdevices):
        if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            devices.append({"id":i,"name": audio.get_device_info_by_host_api_device_index(0, i).get('name'),"active":False})
            print( i, audio.get_device_info_by_host_api_device_index(0, i).get('name'))

    print("-------------------------------------------------------------")
    return devices