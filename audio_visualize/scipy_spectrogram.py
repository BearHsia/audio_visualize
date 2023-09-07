import sounddevice as sd
import numpy as np
import time
import matplotlib.pyplot as plt

class Visualizer():
        def __init__(self):
                self.device = 'Kinect' 
                self.device_info =  sd.query_devices(self.device, 'input')
                print(self.device_info)

                self.display_sec = 5
                self.row_num = self.display_sec*int(self.device_info['default_samplerate'])
                self.display_data = None

                self.stream  = sd.InputStream(device = self.device, 
                                                channels = self.device_info['max_input_channels'], 
                                                samplerate = self.device_info['default_samplerate'], 
                                                callback  = self.audio_callback)

                self.stream.start()

        def audio_callback(self,indata,frames,time,status):
                if self.display_data is None:
                        self.display_data = indata*10
                else:
                self.display_data = np.append(self.display_data,indata*10,axis=0)
                if np.shape(self.display_data)[0]>self.device_info['default_samplerate']:
                        self.display_data = None

        def trim(self):
                self.display_data = np.delete(self.display_data, range(0,np.shape(self.display_data)[0]-self.row_num), axis=0)
                return self.display_data.copy()


if __name__ == '__main__':
        vis = Visualizer()

        fig = plt.figure()
        ax1 = fig.add_subplot(1,1,1)

        while True:
                outdata = vis.trim()
                ax1.specgram(np.flip(outdata[:,0]), Fs = vis.device_info['default_samplerate']) 
                plt.title('matplotlib.pyplot.specgram() Example\n',fontsize = 14, fontweight ='bold')
                fig.canvas.flush_events()
                plt.show(block=False)