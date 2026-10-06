import cv2
import json
import os

def load_tests_from_json(json_file):
    if not(os.path.isfile(json_file)):
        raise Exception("Test file was not found")
    with open(json_file) as readed_file:
        ret =  json.load(readed_file)
    return ret


def identify_device(camera_id=0):
    print("Scan the QR code of the device ...")
    cap = cv2.VideoCapture(camera_id)
    qr_detector = cv2.QRCodeDetector()
    while True:
        _, img = cap.read()
        data, bbox, _ = qr_detector.detectAndDecode(img)
        if(data):
            print(f"[INFO] Device identity is: {data}")
            break
    cap.release()
    return data

