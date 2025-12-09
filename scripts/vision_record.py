import threading
import time
import ntcore
import cv2
import os

from ntcore._ntcore import NetworkTable
from dataclasses import dataclass

@dataclass
class VideoHandler:
    capture: cv2.VideoCapture
    recorder: cv2.VideoWriter

is_sim = True
multithreading_lock = threading.Lock()
handlers: list[VideoHandler] = []

def on_listen(parent: NetworkTable, key: str, child: NetworkTable):
    try:
        streams_topic = child.getStringArrayTopic("streams")
        streams_sub = streams_topic.subscribe([])
        while len(streams_sub.get()) == 0:
            time.sleep(1.0)
        stream_url = streams_sub.get()[0].replace("mjpg:", "")
        stream = cv2.VideoCapture(stream_url)
        print("Stream has initialized at URL " + stream_url)
        frame_width = int(stream.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(frame_height, frame_width)
        fps = 20.0
        fourcc = cv2.VideoWriter.fourcc(*'mp4v')
        file_path = f"../logs/camRecordings/{key}-{time.strftime('%m-%d_%H-%M-%S')}.mp4"
        out = cv2.VideoWriter(file_path, fourcc, fps, (frame_width, frame_height))
        print("Beginning....")
        time.sleep(1)
        if out.isOpened():
            print("Video Recording has started with filename " + key)
            with multithreading_lock:
                handlers.append(VideoHandler(capture=stream, recorder=out))
        else:
            raise Exception("Video Writer did not open")
    except Exception as e:
        print(f"Error: {e}")

def main():
    try:
        try:
            os.mkdir("../logs/camRecordings")
        except FileExistsError:
            pass
        inst = ntcore.NetworkTableInstance.getDefault()
        if is_sim:
            inst.setServer("127.0.0.1", 5810)
        else:
            inst.setServerTeam(5160)
        inst.startClient4("Test Client")
        while not inst.isConnected():
            time.sleep(1.0)
        time.sleep(5)
        t = inst.getTable("CameraPublisher")
        t.addSubTableListener(on_listen)
        print("Network Tables Subscription has initialized.")
        control_data = inst.getTable("FMSInfo").getIntegerTopic("FMSControlData").subscribe(0)
        notif_count = 0
        time.sleep(5)
        while control_data.get() == 0:
            notif_count += 1
            if notif_count == 20:
                print("Waiting for DriverStation to enable...")
            time.sleep(0.1)
        print("Driver Station Enabled; video recording started")
        while True:
            if control_data.get() == 0:
                print("Driver Station Disabled; video recording stopped.")
                break
            for handler in handlers:
                if not handler.capture.isOpened():
                    continue
                ret, frame = handler.capture.read()
                if not ret:
                    continue
                handler.recorder.write(frame)
    finally:
        for handler in handlers:
            handler.capture.release()
            handler.recorder.release()

if __name__ == '__main__':
    main()
