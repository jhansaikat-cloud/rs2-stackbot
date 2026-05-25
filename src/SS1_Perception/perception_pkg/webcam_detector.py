import cv2
import numpy as np

# ── Change this to your webcam device number ──
WEBCAM_INDEX = 3   # try 0, 1, or 2 if it doesn't open

# ── HSV colour ranges ──
# These are starting points — run the HSV tuner to refine them
COLOURS = {
    'red': {
        'lower1': np.array([0,   165,  60]),   # red wraps in HSV
        'upper1': np.array([10,  255, 255]),    # low side of red
        'lower2': np.array([168, 165,  60]),    # your tuned values
        'upper2': np.array([179, 255, 180]),    # high side of red
        'bgr': (0, 0, 255)
    },
    'blue': {
    'lower1': np.array([65,  60,  60]),     # raised S min from 24→60, V min from 34→60
    'upper1': np.array([103, 255, 255]),    # opened S max and V max fully
    'bgr': (255, 0, 0)
    },
    'green': {
        'lower1': np.array([17,  45,  75]),     # your tuned values
        'upper1': np.array([50,  255, 255]),
        'bgr': (0, 200, 0)
    },
    'yellow': {
        'lower1': np.array([12,  139, 125]),    # your tuned values
        'upper1': np.array([23,  214, 155]),
        'bgr': (0, 220, 220)
    },
}

MIN_AREA = 800   # minimum blob size in pixels — reduces noise


def get_mask(hsv, colour):
    mask = cv2.inRange(hsv, colour['lower1'], colour['upper1'])
    if 'lower2' in colour:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(hsv, colour['lower2'], colour['upper2'])
        )
    # Clean up noise
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []

    for name, colour in COLOURS.items():
        mask = get_mask(hsv, colour)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            squareness = round(min(w, h) / max(w, h), 2)
            detections.append({
                'colour':     name,
                'bgr':        colour['bgr'],
                'bbox':       (x, y, w, h),
                'centre':     (cx, cy),
                'area':       int(area),
                'squareness': squareness,
            })
    return detections


def draw(frame, detections):
    for d in detections:
        x, y, w, h = d['bbox']
        cx, cy     = d['centre']
        col        = d['bgr']

        # Bounding box
        cv2.rectangle(frame, (x, y), (x+w, y+h), col, 2)

        # Centre dot
        cv2.circle(frame, (cx, cy), 6, col, -1)

        # Label — colour + squareness score
        cv2.putText(frame, f"{d['colour']} sq:{d['squareness']}",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

        # Pixel coordinates below box
        cv2.putText(frame, f"px({cx},{cy})",
                    (x, y + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    return frame


def main():
    cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"Could not open webcam at index {WEBCAM_INDEX}")
        print("Try changing WEBCAM_INDEX to 0, 1, or 2 at the top of this file")
        return

    # Set to 1080p
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)

    print("Webcam detector running!")
    print("Press Q to quit, S to save a screenshot")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        detections = detect(frame)
        frame      = draw(frame, detections)

        # Detection count top left
        cv2.putText(frame, f"Detected: {len(detections)} object(s)",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        # Print to terminal
        for d in detections:
            print(f"  {d['colour']:6s} | centre px:({d['centre'][0]:4d},{d['centre'][1]:4d}) "
                  f"| area:{d['area']:6d} | squareness:{d['squareness']}")

        cv2.imshow('Webcam Detector', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = 'detection_screenshot.png'
            cv2.imwrite(fname, frame)
            print(f"Screenshot saved as {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
