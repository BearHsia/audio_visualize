import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile
import numpy as np
sample_rate, samples = wavfile.read('audio_subscriber_mic0.wav')
print("sample rate",sample_rate)
print("samples dtype:",type(samples))
print("samples shape:", np.shape(samples))


plt.specgram(samples, Fs = sample_rate) 
plt.title('matplotlib.pyplot.specgram() Example\n',fontsize = 14, fontweight ='bold')
  
plt.show()
