import numpy as np
import librosa
from scipy.io.wavfile import write

y, sr = librosa.load("nic.wav")
y_fast = librosa.effects.time_stretch(y, rate=1.5)
tempo, bf = librosa.beat.beat_track(y=y_fast, sr=sr)
s = str(tempo)
print(np.shape(y_fast))

write('test.wav', sr*2, y_fast)

