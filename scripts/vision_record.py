import threading
import time

import ntcore
import cv2
from ntcore._ntcore import NetworkTable

data_lock = threading.Lock()
topics = []

def on_listen(parent: NetworkTable, key: str, child: NetworkTable):
    t = child.getBooleanTopic("connected").subscribe(False)
    with data_lock:
        topics.append(t)

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    # inst.setServerTeam(5160)
    # inst.startDSClient()
    inst.setServer("127.0.0.1", 5810)
    inst.startClient4("Test Client")
    while not inst.isConnected():
        time.sleep(0.1)
    t = inst.getTable("CameraPublisher")
    t.addSubTableListener(on_listen)
    time.sleep(1)
    print(topics)