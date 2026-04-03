import os
import cv2
import json
import time
import numpy as np
import re
from difflib import SequenceMatcher

# ----------------------------
# Configuration
# ----------------------------

CAMERA_ID = 2
DISPLAY_W = 1280
DISPLAY_H = 720
UPDATE_INTERVAL = 1.0
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_JSON = os.path.join(BASE_DIR, "shelf_viewer", "shelf_order.json")

MIN_WIDTH = 100
MAX_WIDTH = 160
MIN_HEIGHT = 110
MAX_HEIGHT = 180
ALIGNMENT_TOLERATION = 25

MIN_DETECTIONS_TO_CONFIRM = 3
CONFIDENCE_THRESHOLD = 0.75
SIMILARITY_THRESHOLD = 0.85

TAG_COOLDOWN = 10.0  # seconds to ignore tag after a state switch
ACTUATOR_DISPLAY_DURATION = 3.0  # seconds to show actuator message

# ROI as fraction of frame (left, top, right, bottom)
ROI_X0 = 0.10
ROI_Y0 = 0.20
ROI_X1 = 0.90
ROI_Y1 = 0.80

COLORS = [
    (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 165, 255),
    (255, 255, 0), (128, 0, 128), (255, 128, 0), (0, 255, 128),
    (128, 255, 0), (255, 0, 128)
]

# ----------------------------
# State Machine
# ----------------------------

scan_state = "IDLE"
detection_history = []

# ----------------------------
# AprilTag Detection
# ----------------------------

def init_apriltag_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector

def detect_apriltag(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        return True, ids.flatten().tolist(), corners
    return False, [], []

# ----------------------------
# OCR Init
# ----------------------------

def init_easyocr():
    import torch
    import easyocr
    use_gpu = torch.cuda.is_available()
    reader = easyocr.Reader(['en'], gpu=use_gpu, recognizer=True, verbose=False)
    return reader

# ----------------------------
# OCR Post-Processing
# ----------------------------

def fix_common_ocr_errors(text):
    text = str(text).upper().strip()
    corrections = {
        '2I1': '211', '2II': '211', '21I': '211',
        'I74': '174', 'I2': '12', 'I39': '139',
        '2OOI': '2001', '2OO5': '2005', '2OI3': '2013', '2O23': '2023',
        '744.6': '794.6', '794.G': '794.6',
        'OC': 'QC', 'CC': 'QC', '0C': 'QC', 'O0': 'QO',
        'ENOR': 'ENGR', 'FAGR': 'ENGR', 'FNGR': 'ENGR', 'DNOIR': 'ENGR',
        'PHVS': 'PHYS', '@': 'Q',
    }
    for wrong, right in corrections.items():
        text = text.replace(wrong, right)
    text = re.sub(r'I(?=\d)', '1', text)
    text = re.sub(r'(?<=\d)I(?=\d)', '1', text)
    text = re.sub(r'(?<=\d)I\b', '1', text)
    text = re.sub(r'(?<=\d)O(?=\d)', '0', text)
    text = re.sub(r'(?<=\d)O\b', '0', text)
    text = re.sub(r'\b2O', '20', text)
    text = re.sub(r'l(?=\d)', '1', text)
    text = re.sub(r'(?<=\d)l', '1', text)
    text = re.sub(r'(?<=\d)G\b', '6', text)
    text = re.sub(r'\.G\b', '.6', text)
    return text

def normalize_call_number(call_num):
    normalized = " ".join(str(call_num).split()).upper()
    prefixes = ['ENGR', 'MATH', 'PHYS', 'DNOIR', 'ENOR', 'FAGR', 'FNGR', 'SCI', 'TECH']
    words = normalized.split()
    while words and words[0] in prefixes:
        words.pop(0)
    return " ".join(words) if words else normalized

def smart_prepend_single_letter(call_number):
    normalized = normalize_call_number(call_number)
    if re.match(r'^\d', normalized):
        match = re.match(r'^(\d+)', normalized)
        if match:
            num = int(match.group(1))
            if 300 <= num <= 399:
                return 'Q ' + normalized
    return call_number

def extract_core_call_number(call_num):
    normalized = normalize_call_number(call_num)
    match = re.search(r'([A-Z]{1,3}\s*\d+(?:\.\d+)?)', normalized)
    if match:
        core = match.group(1)
        cutter_match = re.search(r'([A-Z]\d+)', normalized[match.end():])
        if cutter_match:
            core += ' ' + cutter_match.group(1)
        return core
    return normalized

def string_similarity(str1, str2):
    return SequenceMatcher(None, str1, str2).ratio()

def books_are_same(call_num1, call_num2):
    core1 = extract_core_call_number(call_num1)
    core2 = extract_core_call_number(call_num2)
    if core1 == core2 and core1:
        return True
    norm1 = normalize_call_number(call_num1)
    norm2 = normalize_call_number(call_num2)
    if string_similarity(norm1, norm2) >= SIMILARITY_THRESHOLD:
        return True
    if len(core1) > 5 and len(core2) > 5:
        if core1 in norm2 or core2 in norm1:
            return True
    return False

def find_matching_book(call_number, existing_books):
    for book in existing_books:
        if books_are_same(call_number, book["call_number"]):
            return book
    return None

def box_area(box):
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))

def box_center(box):
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return sum(xs) / 4, sum(ys) / 4

def box_bounds(box):
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return min(xs), min(ys), max(xs), max(ys)

def cluster_detections_by_book(detections):
    if not detections:
        return []
    detections = sorted(detections, key=lambda d: box_center(d[0])[0])
    clusters = []
    for box, text, conf in detections:
        cx, cy = box_center(box)
        x_min, y_min, x_max, y_max = box_bounds(box)
        placed = False
        for cluster in clusters:
            cluster_width = cluster["x_max"] - cluster["x_min"]
            margin = max(15, int(cluster_width * 0.25))
            horizontal_overlap = not (x_max < cluster["x_min"] - margin or x_min > cluster["x_max"] + margin)
            center_close = abs(cx - cluster["x_pos"]) < cluster_width * 0.6
            if horizontal_overlap and center_close:
                cluster["items"].append((box, text, conf, cx, cy))
                cluster["x_pos"] = sum(i[3] for i in cluster["items"]) / len(cluster["items"])
                cluster["x_min"] = min(cluster["x_min"], x_min)
                cluster["x_max"] = max(cluster["x_max"], x_max)
                cluster["all_boxes"].append(box)
                placed = True
                break
        if not placed:
            clusters.append({
                "x_pos": cx, "x_min": x_min, "x_max": x_max,
                "items": [(box, text, conf, cx, cy)],
                "all_boxes": [box]
            })
    clusters.sort(key=lambda c: c["x_pos"])
    return clusters

def build_call_number_from_cluster(cluster_items):
    sorted_items = sorted(cluster_items, key=lambda item: (item[4], item[3]))
    parts = [fix_common_ocr_errors(text) for _, text, _, _, _ in sorted_items if text]
    full = " ".join(parts)
    full = smart_prepend_single_letter(full)
    return full if full else "UNKNOWN"

def easyocr_to_detections(results):
    dets = []
    for item in results:
        if item is None or len(item) < 3:
            continue
        box, text, conf = item[0], item[1], item[2]
        if not text or not str(text).strip():
            continue
        box = [[float(p[0]), float(p[1])] for p in box]
        dets.append((box, str(text).strip(), float(conf) if conf is not None else 0.0))
    return dets

# ----------------------------
# OCR Preprocess
# ----------------------------

def preprocess_for_ocr(roi_bgr):
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    kernel_sharpen = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    sharpened = cv2.filter2D(gray, -1, kernel_sharpen)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(sharpened)
    bilateral = cv2.bilateralFilter(enhanced, 5, 75, 75)
    kernel_morph = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    morph = cv2.morphologyEx(bilateral, cv2.MORPH_CLOSE, kernel_morph)
    return cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)

# ----------------------------
# Rectangle Detection inside ROI
# ----------------------------

def detect_label_rectangle_in_roi(frame, roi_x0, roi_y0, roi_x1, roi_y1):
    roi = frame[roi_y0:roi_y1, roi_x0:roi_x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 50, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_rect = None
    best_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if 500 < area < 50000:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)
                if (MIN_WIDTH <= w <= MAX_WIDTH) and (MIN_HEIGHT <= h <= MAX_HEIGHT):
                    if area > best_area:
                        best_area = area
                        best_rect = (x, y, w, h)
    if best_rect is None:
        return None
    x, y, w, h = best_rect
    cx = roi_x0 + x + w // 2
    cy = roi_y0 + y + h // 2
    return cx, cy, w, h

# ----------------------------
# OCR on detected rectangle
# ----------------------------

def run_ocr_on_rect(frame, reader, rect):
    cx, cy, rw, rh = rect
    padding = 10
    x0 = max(0, cx - rw // 2 - padding)
    y0 = max(0, cy - rh // 2 - padding)
    x1 = min(frame.shape[1], cx + rw // 2 + padding)
    y1 = min(frame.shape[0], cy + rh // 2 + padding)
    rect_roi = frame[y0:y1, x0:x1].copy()
    roi_proc = preprocess_for_ocr(rect_roi)
    results = reader.readtext(
        roi_proc, detail=1, paragraph=False, min_size=5,
        text_threshold=0.65, low_text=0.4, link_threshold=0.4,
        canvas_size=3200, mag_ratio=1.5, slope_ths=0.3,
    )
    dets = easyocr_to_detections(results)
    adjusted = []
    for box, text, conf in dets:
        adjusted_box = [[p[0] + x0, p[1] + y0] for p in box]
        adjusted.append((adjusted_box, text, conf))
    roi_area = rect_roi.shape[0] * rect_roi.shape[1]
    filtered = []
    for box, text, conf in adjusted:
        area = box_area(box)
        if area < 0.0001 * roi_area or area > 0.35 * roi_area:
            continue
        x_min, y_min, x_max, y_max = box_bounds(box)
        if (y_max - y_min) > (x_max - x_min) * 8.0:
            continue
        filtered.append((box, text, conf))
    return cluster_detections_by_book(filtered)

# ----------------------------
# Detection History
# ----------------------------

def update_detection_history(clusters):
    now = time.time()
    for cluster in clusters:
        call_num = build_call_number_from_cluster(cluster["items"])
        conf_avg = sum(i[2] for i in cluster["items"]) / len(cluster["items"])
        if conf_avg < CONFIDENCE_THRESHOLD:
            continue
        detection_history.append({
            "call_number": call_num,
            "confidence": conf_avg,
            "text_components": [fix_common_ocr_errors(i[1]) for i in cluster["items"]],
            "timestamp": now
        })
    if len(detection_history) > 200:
        detection_history[:] = detection_history[-200:]

def get_stable_shelf_order():
    if not detection_history:
        return []
    unique_books = []
    for det in detection_history:
        call_num = det["call_number"]
        match = find_matching_book(call_num, unique_books)
        if match:
            match["detections"].append(det)
            match["detection_count"] += 1
            confs = [d["confidence"] for d in match["detections"]]
            match["confidence"] = sum(confs) / len(confs)
            all_nums = [d["call_number"] for d in match["detections"]]
            match["call_number"] = max(all_nums, key=lambda x: len(normalize_call_number(x)))
            match["text_components"] = det["text_components"]
        else:
            unique_books.append({
                "call_number": call_num,
                "confidence": det["confidence"],
                "text_components": det["text_components"],
                "detection_count": 1,
                "detections": [det]
            })
    stable = [b for b in unique_books if b["detection_count"] >= MIN_DETECTIONS_TO_CONFIRM]

    def sort_key(book):
        core = extract_core_call_number(book["call_number"])
        m = re.match(r'([A-Z]+)(\d+\.?\d*)', core)
        if m:
            return (m.group(1), float(m.group(2)))
        return (core, 0)

    try:
        stable.sort(key=sort_key)
    except Exception:
        stable.sort(key=lambda x: extract_core_call_number(x["call_number"]))
    for i, book in enumerate(stable):
        book["position"] = i + 1
        book.pop("detections", None)
    return stable

def save_shelf_order(output_json):
    stable_books = get_stable_shelf_order()
    shelf_order = {"num_books": len(stable_books), "books": stable_books}
    with open(output_json, "w") as f:
        json.dump(shelf_order, f, indent=2)
    return stable_books

# ----------------------------
# MAIN
# ----------------------------

def main():
    global scan_state, detection_history

    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)

    reader = init_easyocr()
    tag_detector = init_apriltag_detector()
    cap = cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        print("Camera open failed")
        return

    current_clusters = []
    frame_count = 0
    last_save = time.time()
    last_tag_switch = 0.0
    tag_detection_count = 0          # total number of tag switches
    actuator_message_until = 0.0     # timestamp until which to show actuator message

    print("State: IDLE — waiting for AprilTag to START scanning...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (DISPLAY_W, DISPLAY_H))
        frame_count += 1

        h, w = frame.shape[:2]

        # Compute ROI pixel coords
        rx0, ry0 = int(w * ROI_X0), int(h * ROI_Y0)
        rx1, ry1 = int(w * ROI_X1), int(h * ROI_Y1)

        # --- AprilTag detection ---
        tag_found, tag_ids, tag_corners = detect_apriltag(frame, tag_detector)
        cooldown_elapsed = (time.time() - last_tag_switch) >= TAG_COOLDOWN

        if tag_found and cooldown_elapsed:
            tag_detection_count += 1
            last_tag_switch = time.time()

            if scan_state == "IDLE":
                scan_state = "SCANNING"
                detection_history.clear()
                current_clusters = []
                print(f"AprilTag {tag_ids} detected (#{tag_detection_count}) → START SCANNING")
            elif scan_state == "SCANNING":
                scan_state = "IDLE"
                stable = save_shelf_order(OUTPUT_JSON)
                print(f"AprilTag {tag_ids} detected (#{tag_detection_count}) → STOP SCANNING")
                print(f"Saved {len(stable)} books → {OUTPUT_JSON}")
                for b in stable:
                    print(f"  {b['position']}. {b['call_number']} "
                          f"(x{b['detection_count']}, conf {b['confidence']:.2f})")

            # On every even detection, trigger actuator message
            if tag_detection_count % 2 == 0:
                actuator_message_until = time.time() + ACTUATOR_DISPLAY_DURATION
                print(f"Even detection #{tag_detection_count} → Moving linear actuator UP for {ACTUATOR_DISPLAY_DURATION}s")

        elif tag_found and not cooldown_elapsed:
            remaining = TAG_COOLDOWN - (time.time() - last_tag_switch)
            print(f"Tag seen — cooldown active ({remaining:.1f}s remaining)", end="\r")

        # --- OCR loop (only runs while SCANNING and actuator not running) ---
        detected_rect = None
        actuator_active = time.time() < actuator_message_until

        if scan_state == "SCANNING" and not actuator_active:
            detected_rect = detect_label_rectangle_in_roi(frame, rx0, ry0, rx1, ry1)
            if detected_rect is not None:
                current_clusters = run_ocr_on_rect(frame, reader, detected_rect)
                if current_clusters:
                    update_detection_history(current_clusters)
            if time.time() - last_save > UPDATE_INTERVAL:
                stable = save_shelf_order(OUTPUT_JSON)
                print(f"[SCANNING] {len(stable)} books detected so far")
                last_save = time.time()

        # --- Draw overlay ---
        roi_color = (0, 255, 0) if scan_state == "SCANNING" else (180, 180, 180)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), roi_color, 2)
        cv2.putText(frame, "OCR ROI", (rx0 + 5, ry0 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, roi_color, 1)

        # Draw detected label rectangle in red
        if detected_rect is not None:
            rcx, rcy, rw, rh = detected_rect
            cv2.rectangle(frame, (rcx - rw//2, rcy - rh//2), (rcx + rw//2, rcy + rh//2), (0, 0, 255), 2)
            cv2.circle(frame, (rcx, rcy), 5, (255, 0, 0), -1)
            cv2.putText(frame, "LABEL", (rcx - rw//2, rcy - rh//2 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # OCR cluster boxes
        if scan_state == "SCANNING":
            for idx, cluster in enumerate(current_clusters):
                color = COLORS[idx % len(COLORS)]
                for box in cluster["all_boxes"]:
                    pts = np.array([[int(p[0]), int(p[1])] for p in box], dtype=np.int32)
                    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
                call_num = build_call_number_from_cluster(cluster["items"])
                cx_c = int(cluster["x_pos"])
                cy_c = int(cluster["items"][0][4])
                cv2.putText(frame, call_num[:20], (cx_c, cy_c - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Actuator message overlay — shown for ACTUATOR_DISPLAY_DURATION secs on even detections
        if actuator_active:
            remaining_act = actuator_message_until - time.time()
            msg = f"Moving linear actuator UP... ({remaining_act:.1f}s)"
            # Dark background bar
            cv2.rectangle(frame, (0, h//2 - 50), (w, h//2 + 50), (0, 0, 0), -1)
            cv2.putText(frame, msg, (w//2 - 380, h//2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 3)

        # State label
        state_color = (0, 255, 0) if scan_state == "SCANNING" else (0, 0, 255)
        cv2.putText(frame, f"State: {scan_state}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2)
        cv2.putText(frame, f"Tag count: {tag_detection_count}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        if tag_found:
            cv2.aruco.drawDetectedMarkers(frame, tag_corners)
            cv2.putText(frame, f"Tag IDs: {tag_ids}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        stable_books = get_stable_shelf_order()
        cv2.putText(frame, f"Books: {len(stable_books)}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Shelf OCR + AprilTag", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
