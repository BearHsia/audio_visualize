# Quickly import essential libraries
import rclpy
import numpy as np
from rclpy.node import Node
from audio_msgs.msg import AudioSignal
import threading
import matplotlib.pyplot as plt

class AudioVisualizer(Node):
    def __init__(self):
        super().__init__('audio_visualizer')
        self.subscription = self.create_subscription(
                AudioSignal,
                'audio_signals',
                self.listener_callback,
                10)
        self.display_sec = 10
        self.initialized = False
        self.channels = None
        self.sample_rate = None
        self.row_num = None
        self.display_data = None
        

    def listener_callback(self, msg):
        n_mics = msg.n_mics
        n_buffer = msg.n_buffer

        if not self.initialized:
            self.initialized = True
            self.channels = n_mics
            self.sample_rate = msg.fs
            self.row_num = 5*self.sample_rate
            self.display_data = np.zeros((self.row_num,self.channels))

        indata = np.array(msg.signals_vect).reshape((n_buffer, n_mics))
        self.display_data = np.append(self.display_data,indata,axis=0)
        self.get_logger().info('data shape: "%s"' % str(np.shape(self.display_data)))
        self.display_data = np.delete(self.display_data, range(0,n_buffer), axis=0)

    def trim(self):
        return self.display_data.copy()

def main(args=None):

    rclpy.init(args=args)

    audio_visualizer = AudioVisualizer()
    
    #rclpy.spin(audio_visualizer)
    thread = threading.Thread(target=rclpy.spin, args=(audio_visualizer,), daemon=True)
    thread.start()

    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    #rate = audio_visualizer.create_rate(10)
    try:
    	while rclpy.ok():
            if audio_visualizer.initialized:
                outdata = audio_visualizer.trim()
                ax1.specgram(np.flip(outdata[:,0]), Fs = audio_visualizer.sample_rate)
                #plt.title('matplotlib.pyplot.specgram() Example\n',fontsize = 14, fontweight ='bold')
                fig.canvas.flush_events()
                plt.show(block=False)
            #rate.sleep()
    except KeyboardInterrupt:
        pass


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    audio_visualizer.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
