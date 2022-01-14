
import requests
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning

requests.packages.urllib3.disable_warnings(InsecureRequestWarning)


import socket 
import struct
import rospy
import sys
from geometry_msgs.msg           import Pose, PoseStamped, PoseArray



from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 

rospy.init_node('ip_camera', anonymous=True)

image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
bridge = CvBridge()

# # Replace the below URL with your own. Make sure to add "/shot.jpg" at last.
url = "https://192.168.31.104:8080/shot.jpg"
url_vid = 'http://192.168.31.104:8080/video'




import cv2 
import numpy as np
import imutils

url = "https://10.0.0.60:8080/shot.jpg"

#/settings/quality?set=XX (where XX = quality %, application/xml)
# torch_resp = requests.get("http://10.0.0.60:8080/enabletorch", verify = False, auth=("pranav", "heeralal"))

# While loop to continuously fetching data from the Url
while True:
    img_resp=0
    img_resp = requests.get(url, verify = False, auth=("pranav", "heeralal"))
    print(img_resp)

    
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    
    img = cv2.imdecode(img_arr, -1)
    # img = imutils.resize(img, width=1000, height=1800)
    print(img.size)

    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    img_msg.header.stamp = rospy.get_rostime()
    image_pub.publish(img_msg)


    cv2.imshow("Android_cam", img)
  
    # Press Esc key to exit
    if cv2.waitKey(1) == 27:
        break
  
cv2.destroyAllWindows()














# UDP_IP = "127.0.0.1"
# UDP_PORT = 60010
 
# sock = socket.socket(socket.AF_INET, # Internet
#               socket.SOCK_DGRAM) # UDP

# sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

# sock.bind(('', UDP_PORT))
# #sock.connect(('192.168.0.243',UDP_PORT))


# #sock.send('0:0',('192.168.43.212',UDP_PORT)) 


# rospy.init_node('PosePublisher',anonymous=True)
# publc=rospy.Publisher('leftControllerPose',PoseStamped,queue_size=100)
# pubrc=rospy.Publisher('rightControllerPose',PoseStamped,queue_size=100)
# pubhead=rospy.Publisher('headPose',PoseStamped,queue_size=100)
# pubtrac1=rospy.Publisher('trackerPose1',PoseStamped,queue_size=100)
# pubtrac2=rospy.Publisher('trackerPose2',PoseStamped,queue_size=100)


# pubobs=rospy.Publisher('obsPose',PoseStamped,queue_size=100)

# trac_1_z=0

# print("Starting Receiving")


# print()
# while True :
# 	print("Trying to recv")

# 	raw_data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
# 	#print(raw_data,addr)
# 	#print(sys.getsizeof(raw_data))
# 	raw_data=struct.unpack('fffffffi', raw_data)
# 	print( raw_data )

# 	poseStamped=PoseStamped()
# 	poseStamped.header.frame_id=str(raw_data[7])

# 	poseStamped.pose.position.x=(raw_data[0])
# 	poseStamped.pose.position.y=(raw_data[1])
# 	poseStamped.pose.position.z=(raw_data[2])

# 	poseStamped.pose.orientation.x=(raw_data[3])
# 	poseStamped.pose.orientation.y=(raw_data[4])
# 	poseStamped.pose.orientation.z=(raw_data[5])
# 	poseStamped.pose.orientation.w=(raw_data[6])

# 	frame_id=raw_data[7]
# 	if frame_id==0:
# 		pubhead.publish(poseStamped)

# 	if frame_id==1:
# 		publc.publish(poseStamped)

# 	if frame_id==2:
# 		pubrc.publish(poseStamped)

# 	if frame_id==3:
# 		pubtrac1.publish(poseStamped)
# 		trac_1_z=poseStamped.pose.position.z

# 	if frame_id==4:
# 		poseStamped.pose.position.z=trac_1_z
# 		pubtrac2.publish(poseStamped)


# 	if frame_id==5:
# 		pubobs.publish(poseStamped)





