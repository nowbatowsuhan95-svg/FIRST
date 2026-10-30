"""Camera fire detector that emails an alert with a snapshot via Gmail.

Detection = fire-coloured pixels (HSV) + flicker (the fire mask keeps changing
between frames) sustained over several consecutive frames. This is a heuristic
and is NOT a substitute for a real smoke/fire alarm.

Usage:  python fire_alert.py [--camera 0 | --video file.mp4] [--no-gui] [--test-email]
"""
from __future__ import annotations

import argparse
import os
import smtplib
import ssl
import threading
import time
from collections import deque
from datetime import datetime
from email.message import EmailMessage
from pathlib import Path

import cv2
import numpy as np

# ---- tunables -------------------------------------------------------------
HSV_LOW = np.array([0, 110, 200], np.uint8)     # bright, saturated red/orange/yellow
HSV_HIGH = np.array([35, 255, 255], np.uint8)
MIN_FIRE_RATIO = 0.004      # fraction of frame that must look like fire
MIN_FLICKER_RATIO = 0.15    # fraction of fire pixels that changed since last frame
WINDOW = 10                 # frames considered
REQUIRED_HITS = 7           # positives needed within WINDOW to raise alarm
COOLDOWN_S = 300            # min seconds between emails
PROC_WIDTH = 480            # downscale for speed

SNAP_DIR = Path(__file__).parent / "snapshots"


def load_env(path: Path) -> None:
    """Minimal .env loader (no extra dependency)."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


class FireDetector:
    def __init__(self) -> None:
        self.prev_mask: np.ndarray | None = None
        self.hits: deque[bool] = deque(maxlen=WINDOW)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def update(self, frame: np.ndarray) -> tuple[bool, np.ndarray, float]:
        """Returns (alarm, fire_mask, fire_ratio) for a BGR frame."""
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (PROC_WIDTH, int(h * PROC_WIDTH / w)))
        blur = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.dilate(mask, self.kernel)

        fire_px = int(cv2.countNonZero(mask))
        ratio = fire_px / mask.size

        flicker = 0.0
        if self.prev_mask is not None and fire_px:
            changed = cv2.countNonZero(cv2.bitwise_xor(mask, self.prev_mask))
            flicker = changed / fire_px
        self.prev_mask = mask

        self.hits.append(ratio >= MIN_FIRE_RATIO and flicker >= MIN_FLICKER_RATIO)
        alarm = sum(self.hits) >= REQUIRED_HITS
        return alarm, mask, ratio


def send_email(jpeg: bytes, ratio: float) -> None:
    user = os.environ.get("GMAIL_USER")
    pwd = os.environ.get("GMAIL_APP_PASSWORD")
    to = os.environ.get("ALERT_TO", "nowbatowsuhan95@gmail.com")
    if not user or not pwd:
        print("[email] GMAIL_USER / GMAIL_APP_PASSWORD not set; alert NOT sent")
        return
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    msg = EmailMessage()
    msg["Subject"] = f"FIRE ALERT - possible fire detected ({now})"
    msg["From"] = user
    msg["To"] = to
    msg.set_content(
        f"Possible fire detected by camera at {now}.\n"
        f"Fire-like area: {ratio:.1%} of frame.\n"
        "Snapshot attached. This is an automated heuristic - verify and call "
        "emergency services if real."
    )
    msg.add_attachment(jpeg, maintype="image", subtype="jpeg", filename="fire.jpg")
    try:
        with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=ssl.create_default_context()) as s:
            s.login(user, pwd)
            s.send_message(msg)
        print(f"[email] alert sent to {to}")
    except Exception as e:  # network/auth errors must not kill the camera loop
        print(f"[email] FAILED: {e}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", help="video file instead of a camera")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--test-email", action="store_true", help="send a test email and exit")
    args = ap.parse_args()

    load_env(Path(__file__).parent / ".env")

    if args.test_email:
        ok, buf = cv2.imencode(".jpg", np.zeros((120, 160, 3), np.uint8))
        send_email(buf.tobytes(), 0.0)
        return

    cap = cv2.VideoCapture(args.video if args.video else args.camera)
    if not cap.isOpened():
        raise SystemExit("Cannot open video source")

    det = FireDetector()
    last_sent = 0.0
    SNAP_DIR.mkdir(exist_ok=True)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        alarm, mask, ratio = det.update(frame)

        if alarm and time.time() - last_sent > COOLDOWN_S:
            last_sent = time.time()
            shot = frame.copy()
            cv2.putText(shot, "FIRE?", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            ok_enc, buf = cv2.imencode(".jpg", shot)
            if ok_enc:
                (SNAP_DIR / f"fire_{int(last_sent)}.jpg").write_bytes(buf.tobytes())
                threading.Thread(target=send_email, args=(buf.tobytes(), ratio), daemon=True).start()

        if not args.no_gui:
            view = frame.copy()
            if alarm:
                cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), (0, 0, 255), 6)
                cv2.putText(view, "FIRE DETECTED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            cv2.imshow("Fire detector (q to quit)", view)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
