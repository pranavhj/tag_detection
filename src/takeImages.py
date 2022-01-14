import cv2 
import numpy as np
import imutils
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning

requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

url = "https://10.0.0.60:8080/shot.jpg"

#/settings/quality?set=XX (where XX = quality %, application/xml)
# torch_resp = requests.get("http://10.0.0.60:8080/enabletorch", verify = False, auth=("pranav", "heeralal"))

# While loop to continuously fetching data from the Url
counter=1
while True:
    img_resp=0
    img_resp = requests.get(url, verify = False, auth=("pranav", "heeralal"))
    print(img_resp)

    
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    
    img = cv2.imdecode(img_arr, -1)
    # img = imutils.resize(img, width=1000, height=1800)
    print(img.size)
    
    cv2.imshow("Android_cam", img)
    pressedKey = cv2.waitKey(1) & 0xFF
    if pressedKey == ord('q'):
        cv2.imwrite("images/"+str(counter)+".jpg",img)
        counter+=1
    elif pressedKey == ord('w'):
        break
  
cv2.destroyAllWindows()