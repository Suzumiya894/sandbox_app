import rospy
from sensor_msgs.msg import Image
import cv2
from pyzbar.pyzbar import decode
import requests

import json
import uuid
import os
import traceback

from utils.convert import ImageConverter
from utils.get_euler_angle import LIMO
from cv_bridge import CvBridge

URL = "http://47.101.169.122:8762/vision/detect_traffic_light_color"
IMG_PATH = "/home/agilex/sandbox_app/images/"

img_cvt = ImageConverter()
cv_bridge = CvBridge()
limo = LIMO()

# rospy.init_node("image_retriever", anonymous=True)

COLOR = None
SIZE = None
LIFTER_QRCODE = None
SIZE_THRESHOLD = 2000
DESTINATION_MARK = "destination"


def __retrieve_iamge(compression=None):
    rgb_Image = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout=1)
    # height, width, rgb_data = rgb_Image.height, rgb_Image.width, rgb_Image.data
    img_name = uuid.uuid1().hex
    ## reorganize rgb img data
    cv_img = cv_bridge.imgmsg_to_cv2(rgb_Image, desired_encoding="passthrough") # RGB

    if compression is not None:
        cv_img = img_cvt.compress_image(cv_img, ratio=compression)

    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
    img_path = "{}.{}".format(IMG_PATH + img_name, "png")
    cv2.imwrite(img_path, cv_img) # cv2: BGR saved as RGB

    return img_path


def __query_image(img_path):
    enc_img = img_cvt.encode_from_path(img_path)
    rsp = requests.post(url=URL, data={"image": enc_img})
    rsp = json.loads(rsp.text)
    return rsp


def __output_variables():
    print("color={}, size={}, threshold={}".format(COLOR, SIZE, SIZE_THRESHOLD))


def __delete_image(img_path):
    os.remove(img_path)


def __decode_qrcode(img_path):
    img = cv2.imread(img_path)
    qr_decoded = decode(img)

    if len(qr_decoded) == 0:
        return None
    return qr_decoded[0].data


def scan():
    img_path = __retrieve_iamge(compression=0.5)
    rsp = __query_image(img_path)
    __delete_image(img_path)

    try:
        global COLOR
        global SIZE

        data = rsp["data"]
        COLOR = data["color"]
        SIZE = data["size"]
    except Exception as e:
        print(e.args)
        print(traceback.format_exc())


def is_traffic_light_exist():
    return COLOR is not None


def is_traffic_light_green():
    return COLOR == "green"


def is_distance_safe():
    print("safe size={}".format(SIZE))
    return SIZE is not None and SIZE <= SIZE_THRESHOLD


def is_destination_reached():
    img_path = __retrieve_iamge(compression=None)
    decoded = __decode_qrcode(img_path)
    __delete_image(img_path)
    return decoded == DESTINATION_MARK 


def scan_lifter():
    img_path = __retrieve_iamge(compression=None)
    global LIFTER_QRCODE
    LIFTER_QRCODE = __decode_qrcode(img_path)
    __delete_image(img_path)


def is_lifter_exist():
    return LIFTER_QRCODE is not None

def get_pitch_angle():
    return limo.GetIMUPitchData()

def get_yaw_angle():
    return limo.GetIMUYawData()

def get_roll_angle():
    return limo.GetIMURollData()


if __name__ == "__main__":
    '''
    decode qrcode
    '''
    ## 0
    # # img = cv2.imread("/home/agilex/sandbox_app/images/dest.png")
    # img = cv2.imread("/home/agilex/sandbox_app/images/c6807b8e2aad11ee80f3845cf327d053.png")
    # qr_decoded = decode(img)

    # data = qr_decoded[0].data
    # print(data == "destination")

    ## 1
    # img_path = __retrieve_iamge(compression=None)
    # decoded = __decode_qrcode(img_path)
    # print(decoded)



    '''
    test perception functions
    '''
    while (True):
        op = raw_input("input operation:")

        if op == "exit":
            break
        elif op == "scan":
            scan()
        elif op == "light?":
            print(is_traffic_light_exist())
        elif op == "green?":
            print(is_traffic_light_green())
        elif op == "safe?":
            print(is_distance_safe())
        elif op == "o":
            __output_variables()
        elif op == "dest?":
            print(is_destination_reached())
        else:
            print("{}Invalid operation '{}'{}".format("\033[0;31;47m", op, "\033[0m"))
