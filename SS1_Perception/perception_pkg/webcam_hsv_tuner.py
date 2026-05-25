import cv2
import numpy as np

WEBCAM_INDEX = 3  # change if needed

def main():
    cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Cannot open webcam {WEBCAM_INDEX}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    cv2.namedWindow('HSV Tuner')
    cv2.createTrackbar('H min', 'HSV Tuner',   0, 179, lambda x: None)
    cv2.createTrackbar('H max', 'HSV Tuner', 179, 179, lambda x: None)
    cv2.createTrackbar('S min', 'HSV Tuner',   0, 255, lambda x: None)
    cv2.createTrackbar('S max', 'HSV Tuner', 255, 255, lambda x: None)
    cv2.createTrackbar('V min', 'HSV Tuner',   0, 255, lambda x: None)
    cv2.createTrackbar('V max', 'HSV Tuner', 255, 255, lambda x: None)

    print("Adjust sliders until only your object appears white in the Mask window")
    print("Values are printed live — copy them into webcam_detector.py")
    print("Press Q to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h_min = cv2.getTrackbarPos('H min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V max', 'HSV Tuner')

        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask   = cv2.inRange(hsv,
                             np.array([h_min, s_min, v_min]),
                             np.array([h_max, s_max, v_max]))
        result = cv2.bitwise_and(frame, frame, mask=mask)

        print(f"\rH:[{h_min:3d}-{h_max:3d}]  "
              f"S:[{s_min:3d}-{s_max:3d}]  "
              f"V:[{v_min:3d}-{v_max:3d}]", end='')

        cv2.imshow('HSV Tuner',            frame)
        cv2.imshow('Mask (white=detected)', mask)
        cv2.imshow('Result',               result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

