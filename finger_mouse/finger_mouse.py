"""
Finger Mouse - control the desktop cursor with your index finger.

- Point with your index finger: the cursor follows the fingertip.
- "Push" your finger toward the screen/camera (like pressing a button):
  the item under the cursor is opened (double-click).
- Make a fist (fold your index finger): the cursor freezes, so you can
  reposition your hand without moving the mouse.
- Optional: pinch thumb + index finger for a single click (select).

Press 'q' or ESC in the preview window to quit.

Works on Python 3.9 - 3.14.
Requirements: pip install -r requirements.txt
The hand-tracking model (~8 MB) is downloaded next to this file on first run.
"""

import time
import urllib.request
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import pyautogui
from mediapipe.tasks.python import BaseOptions, vision

# ---------------------------------------------------------------- settings
CAMERA_INDEX = 0
CAM_WIDTH, CAM_HEIGHT = 640, 480

# Only the inner part of the camera image is mapped to the whole screen,
# so you can reach the screen edges without your finger leaving the frame.
FRAME_MARGIN = 100          # pixels trimmed from each side of the camera image

SMOOTHING = 5               # higher = smoother but laggier cursor

# Push detection: the hand looks bigger when it moves toward the camera.
PUSH_WINDOW_S = 0.35        # how quickly the push must happen (seconds)
PUSH_SCALE_RATIO = 1.12     # hand must grow by 12% inside that window
PUSH_COOLDOWN_S = 1.0       # minimum time between two "opens"
CURSOR_LOOKBACK_S = 0.25    # click where the cursor was *before* the push

PINCH_CLICK = True          # thumb+index pinch = single click
PINCH_DISTANCE = 0.05       # normalized distance that counts as a pinch

pyautogui.PAUSE = 0         # no built-in delay after each pyautogui call
pyautogui.FAILSAFE = True   # slam the mouse into a corner to abort

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/latest/hand_landmarker.task")
MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")

# MediaPipe hand landmark indices
WRIST, THUMB_TIP = 0, 4
INDEX_PIP, INDEX_TIP = 6, 8
MIDDLE_MCP = 9


class PushDetector:
    """Detects a quick forward 'push' from the change in apparent hand size."""

    def __init__(self, window_s=PUSH_WINDOW_S, ratio=PUSH_SCALE_RATIO,
                 cooldown_s=PUSH_COOLDOWN_S):
        self.window_s = window_s
        self.ratio = ratio
        self.cooldown_s = cooldown_s
        self.history = deque()          # (timestamp, hand_scale)
        self.last_push = 0.0

    def reset(self):
        self.history.clear()

    def update(self, scale, now):
        self.history.append((now, scale))
        while self.history and now - self.history[0][0] > self.window_s:
            self.history.popleft()

        if now - self.last_push < self.cooldown_s:
            return False

        smallest = min(s for _, s in self.history)
        if smallest > 0 and scale / smallest >= self.ratio:
            self.last_push = now
            # Start fresh so pulling the hand back doesn't trigger again.
            self.history.clear()
            return True
        return False


def distance(a, b):
    return np.hypot(a.x - b.x, a.y - b.y)


def index_is_up(lm):
    # Fingertip above the middle joint (image y grows downward).
    return lm[INDEX_TIP].y < lm[INDEX_PIP].y


def load_hand_tracker():
    if not MODEL_PATH.exists():
        print("Downloading hand-tracking model (first run only)...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)

    options = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.7,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,
    )
    return vision.HandLandmarker.create_from_options(options)


def main():
    screen_w, screen_h = pyautogui.size()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    if not cap.isOpened():
        raise SystemExit("Could not open the webcam.")

    hands = load_hand_tracker()
    last_timestamp_ms = 0

    push = PushDetector()
    cursor_trail = deque()              # (timestamp, x, y) of recent cursor positions
    smooth_x, smooth_y = pyautogui.position()
    pinching = False
    status, status_until = "", 0.0

    print("Finger Mouse running - press 'q' or ESC in the window to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)  # mirror so movement feels natural
            h, w = frame.shape[:2]
            now = time.time()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Video mode needs strictly increasing timestamps.
            timestamp_ms = max(int(time.monotonic() * 1000), last_timestamp_ms + 1)
            last_timestamp_ms = timestamp_ms
            result = hands.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp_ms)

            cv2.rectangle(frame, (FRAME_MARGIN, FRAME_MARGIN),
                          (w - FRAME_MARGIN, h - FRAME_MARGIN), (255, 0, 255), 2)

            if result.hand_landmarks:
                lm = result.hand_landmarks[0]
                vision.drawing_utils.draw_landmarks(
                    frame, lm, vision.HandLandmarksConnections.HAND_CONNECTIONS)

                if index_is_up(lm):
                    # 1) Move the cursor with the index fingertip.
                    fx, fy = lm[INDEX_TIP].x * w, lm[INDEX_TIP].y * h
                    target_x = np.interp(fx, (FRAME_MARGIN, w - FRAME_MARGIN), (0, screen_w - 1))
                    target_y = np.interp(fy, (FRAME_MARGIN, h - FRAME_MARGIN), (0, screen_h - 1))
                    smooth_x += (target_x - smooth_x) / SMOOTHING
                    smooth_y += (target_y - smooth_y) / SMOOTHING
                    # Stay 1px away from the corners so FAILSAFE isn't tripped by accident.
                    cx = int(np.clip(smooth_x, 1, screen_w - 2))
                    cy = int(np.clip(smooth_y, 1, screen_h - 2))
                    pyautogui.moveTo(cx, cy)

                    cursor_trail.append((now, cx, cy))
                    while cursor_trail and now - cursor_trail[0][0] > CURSOR_LOOKBACK_S:
                        cursor_trail.popleft()

                    cv2.circle(frame, (int(fx), int(fy)), 10, (0, 255, 0), cv2.FILLED)

                    # 2) Push toward the camera = open (double-click).
                    hand_scale = distance(lm[WRIST], lm[MIDDLE_MCP])
                    if push.update(hand_scale, now):
                        # The fingertip drifts during a push, so click where
                        # the cursor was just before the push started.
                        _, px, py = cursor_trail[0]
                        pyautogui.doubleClick(px, py)
                        smooth_x, smooth_y = px, py
                        status, status_until = "OPEN!", now + 0.8

                    # 3) Optional pinch = single click (select).
                    if PINCH_CLICK:
                        is_pinch = distance(lm[THUMB_TIP], lm[INDEX_TIP]) < PINCH_DISTANCE
                        if is_pinch and not pinching:
                            pyautogui.click()
                            status, status_until = "click", now + 0.5
                        pinching = is_pinch
                else:
                    # Index folded: freeze the cursor.
                    push.reset()
                    cursor_trail.clear()
                    cv2.putText(frame, "paused", (10, h - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                push.reset()
                cursor_trail.clear()

            if now < status_until:
                cv2.putText(frame, status, (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            cv2.imshow("Finger Mouse", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cap.release()
        hands.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
