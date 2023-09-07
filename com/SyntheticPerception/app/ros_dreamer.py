import message_filters
from sensor_msgs.msg import Image, CameraInfo
from ultralytics import YOLO 
from PIL import Image

img_topic_name = "/robot1/D455_1/color/camera_info"
depth_topic_name = "/robot1/D455_1/depth/camera_info"

MODEL = YOLO("")
DREAMER = None
def get_distane_of_obj(image, depth):
    results - MODEL(image)

    # extract location and get distance using depth

    return distance

def send_spot_velocity_command(linear, angular):
    #publish the spot veloc here
    pass
def callback(image, camera_info):
    # Solve all of perception here...

    # detect object and get dist
    distance = get_distane_of_obj(image,depth)

    # reformat image
    image_for_dreamer = image.resize((64,64))
    depth_for_dreamer = depth

    # get reply for dreamer
    results = DREAMER()

    # extract actions and such
    linear, angular = results

    send_spot_velocity_command(linear, angular)






image_sub = message_filters.Subscriber('image', Image)
info_sub = message_filters.Subscriber('camera_info', CameraInfo)




ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()
