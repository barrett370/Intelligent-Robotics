import speech_recognition as sr
r = sr.Recognizer()
harvard = sr.AudioFile('recordedFile.wav')
with harvard as source:
    audio = r.record(source)
    print(type(audio))
    print(r.recognize_google(audio_data=audio))
    # throws an UnknownValueError when no speech is detected
