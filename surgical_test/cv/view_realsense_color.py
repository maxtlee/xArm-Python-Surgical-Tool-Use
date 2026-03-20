import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    pipeline.start(config)

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense Color", image)

        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            break

finally:
    try:
        pipeline.stop()
    except Exception as e:
        print("pipeline.stop() warning:", e)
    cv2.destroyAllWindows()