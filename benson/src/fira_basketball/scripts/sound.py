import os 
import playsound
from gtts import gTTS
import time
def speak(text):
    tts = gTTS(text = text, lang = 'en')
    filename = 'abc.mp3'
    tts.save(filename)
    # playsound.playsound(filename, False)
    playsound.playsound(filename)
    # print('...')
    os.remove(filename)
    #time.sleep(5)
#speak('hi')

