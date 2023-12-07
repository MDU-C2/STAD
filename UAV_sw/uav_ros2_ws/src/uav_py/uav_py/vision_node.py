
#!/usr/bin/python3
from rclpy.node import Node
import math
import rclpy
import signal
import sys
from roboflowoak import RoboflowOak
from roboflowoak import Prediction
from uav_msgs.msg import Vision
import cv2
import time


class VisionNode(Node):
    def __init__(self):
        print("Starting vision_node...")
        super().__init__("vision_node")
        print("Creating vision_topic publisher...")
        self.publisher = self.create_publisher(Vision, "vision_topic", 10)
        self.create_timer(0.5, self.publish_msg)
    	
    def publish_msg(self, msg: Vision):
        self.publisher.publish(msg)
        print("Published: ", str(msg))



# exit on: CTRL + C
def sigint_callback(sig, frame):
	print("\n")
	print("Shutting down vision_node...")
	rclpy.shutdown()
	sys.exit(0)



class Predictor:
    def __init__(self):
        print("Starting model...")
        self.model = RoboflowOak(
            model = "landing-pad-test-2", 
            confidence = 0.8, 
            overlap = 0.5,
            version = "5", 
            api_key = "3dyKguLbaut4uUdSJ8px", 
            rgb = True,    
            depth = False, 
            device = None, 
            blocking = True)
        
        #print("asdassadsdaasdsda")

        self.red = []
        self.blue = []
        self.result = Vision()
        #print(str(self.result))
        #while True:
        #    pass


    def predict(self) -> bool:

        # Detection
        #result, frame, raw_frame, depth = self.model.detect()
        result, _, _, _ = self.model.detect()
        
        predictions = result["predictions"]

        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from your OAK
        #depth - depth map for raw_frame, center-rectified to the center camera

        return_value = False

        if (predictions):
            blue_found = False
            red_found = False
            
            for p in predictions:
                if (p.class_name == 'blue'):
                    blue_found = True
                    self.blue = p
                elif (p.class_name == 'red'):
                    red_found = True
                    self.red = p

            if (blue_found and red_found):
                #start_point = (int(self.blue.x), int(self.blue.y))
                #end_point = (int(self.red.x), int(self.red.y))
                #color = (0, 0, 0)
                #frame = cv2.arrowedLine(frame, start_point, end_point, color, thickness = 4, tipLength = 0.3)

                #vec_target = Vector(self.blue.x - self.red.x, self.red.y - self.blue.y, 0)
                #vec_up = Vector(0, -1, 0)
                #angle = vec_up.angle_with(vec_target)
                #print("Angle: ", angle)

                # create prediction result
                #self.result.x = int((self.blue.x + self.red.x) / 2)
                #self.result.y = int((self.blue.y + self.red.y) / 2)
                
                # Calculate Depth
                # Note: we do not use depth value from camera because it sucks balls
                #width = (self.blue.width + self.red.width) / 2
                #height = (self.blue.height + self.red.height) / 2
                #size = max(width, height)
                #self.result.depth = 50.0 / size #int(size)

                #self.result.angle = angle

                self.result.red_x = int(self.red.x)
                self.result.red_y = int(self.red.y)
                self.result.red_width = int(self.red.width)
                self.result.red_height = int(self.red.height)
                self.result.blue_x = int(self.blue.x)
                self.result.blue_y = int(self.blue.y)
                self.result.blue_width = int(self.blue.width)
                self.result.blue_height = int(self.blue.height)

                return_value = True
            
            # Show frame
            #cv2.imshow("frame", frame)

        return return_value
    
    def get_prediction(self) -> Vision:
        return self.result
        


def main(args=None):
    signal.signal(signal.SIGINT, sigint_callback)
    
    rclpy.init(args=args)
    
    node = VisionNode()
    predictor = Predictor()

    t0 = time.time()

    while True:
        prediction_ok = predictor.predict()
        if (prediction_ok):
            msg = predictor.get_prediction()
            #print("get")
            t = time.time() - t0
            t0 = t

            msg.time_seconds = t #int((t - int(t)) * 1000)
            #print("bef")
            node.publish_msg(msg)
            #print("aft")
			
    rclpy.shutdown()



if __name__ == "__main__":
	main()
