# Quickly import essential libraries
import rclpy
import numpy as np
from rclpy.node import Node
from audio_msgs.msg import AudioSignal
import threading
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from scipy import signal
from scipy.fft import fftshift
import cv2
fig = plt.figure()
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
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'Specgram', 10)
        #timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        n_mics = msg.n_mics
        n_buffer = msg.n_buffer

        if not self.initialized:
            self.initialized = True
            self.channels = n_mics
            self.sample_rate = msg.fs
            self.row_num = 5*self.sample_rate
            self.display_data = np.zeros((self.row_num,))

        indata = np.array(msg.signals_vect).reshape((n_buffer, n_mics))[:,0]
        #self.get_logger().info('indata shape: "%s"' % str(np.shape(indata)))
        self.display_data = np.append(self.display_data,indata,axis=0)
        self.display_data = self.display_data[n_buffer:]
        

    def timer_callback(self):
        ax1 = fig.add_subplot(1,1,1)
        _,_,_,im = ax1.specgram(np.flip(self.display_data), Fs = self.sample_rate)
        
        #fig.canvas.flush_events()
        plt.show(block=False)
        #r = plt.gcf().canvas.get_renderer()
        #color = im.make_image(r, magnification=2.0)[0]
        #self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv2.flip(color[:,:,:3], 0), 'rgb8'))
        
        canvas = fig.canvas
        canvas.draw()
        image_flat = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')  # (H * W * 3,)
        # NOTE: reversed converts (W, H) from get_width_height to (H, W)
        image = image_flat.reshape(*reversed(canvas.get_width_height()), 3)  # (H, W, 3)
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(image, 'rgb8'))
        
        
        
        
        
    def get(self):
        return self.display_data.copy()

def main(args=None):

    rclpy.init(args=args)

    audio_visualizer = AudioVisualizer()
    
    #rclpy.spin(audio_visualizer)
    thread = threading.Thread(target=rclpy.spin, args=(audio_visualizer,), daemon=True)
    thread.start()

    # fig = plt.figure()
    # ax1 = fig.add_subplot(1,1,1)

    rate = audio_visualizer.create_rate(10)
    try:
        while rclpy.ok():
            if audio_visualizer.initialized:
                #outdata = audio_visualizer.get()
                ax1 = fig.add_subplot(1,1,1)
                _,_,_,im = ax1.specgram(np.flip(audio_visualizer.display_data), Fs = audio_visualizer.sample_rate)
                
                #fig.canvas.flush_events()
                plt.show(block=False)
                #r = plt.gcf().canvas.get_renderer()
                #color = im.make_image(r, magnification=2.0)[0]
                #self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv2.flip(color[:,:,:3], 0), 'rgb8'))
                
                canvas = fig.canvas
                canvas.draw()
                image_flat = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')  # (H * W * 3,)
                # NOTE: reversed converts (W, H) from get_width_height to (H, W)
                image = image_flat.reshape(*reversed(canvas.get_width_height()), 3)  # (H, W, 3)
                audio_visualizer.publisher_.publish(audio_visualizer.bridge.cv2_to_imgmsg(image, 'rgb8'))
            rate.sleep()
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
