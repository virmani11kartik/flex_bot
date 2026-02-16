import os
import cv2
import json
import time
import numpy as np
import re
from difflib import SequenceMatcher


TARGET_FPS = 30.0     # pace preview + debug recording (match capture target)
OCR_HZ = 3.0          # run OCR at this rate (independent of TARGET_FPS)
DRAIN_FRAMES = 0      # IMPORTANT: set to 0 (draining causes time-lapse speed-up)
PREVIEW_MAX_W = 1280  # preview window scale only (recording stays full-res)


# ----------------------------
# Configuration
# ----------------------------
VIDEO_PATH = "0"  # "0" or 0 for webcam, or path to video file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_JSON = os.path.join(BASE_DIR, "shelf_viewer", "shelf_order.json")
UPDATE_INTERVAL = 1.0  # seconds

# ROI Configuration (normalized)
ROI_X0, ROI_Y0 = 0.10, 0.42
ROI_X1, ROI_Y1 = 0.95, 0.77

# Detection stability settings
MIN_DETECTIONS_TO_CONFIRM = 3
CONFIDENCE_THRESHOLD = 0.75
SIMILARITY_THRESHOLD = 0.85

# Headless / display handling
HEADLESS = False

# Debug video
SAVE_DEBUG_VIDEO = True
DEBUG_VIDEO_PATH = os.path.join(BASE_DIR, "debug_overlay.mp4")

# ----------------------------
# NEW: Timing + Camera Controls (recommended)
# ----------------------------
# Keep high-res capture for OCR, but pace streaming/recording to a sane FPS.
CAPTURE_W, CAPTURE_H = 3840, 2160   # try 4K; if it fails, use 1920x1080
TARGET_FPS = 15.0                   # 4K commonly stable at 15; try 30 for 1080p
OCR_HZ = 3.0                        # run OCR 3 times/sec (tune 1–5)

# Drop buffered frames so you always operate on the newest frame (prevents fast-forward feel)
DRAIN_FRAMES = 2

# Display scaling: do NOT shrink recorded video, only shrink the preview window
PREVIEW_MAX_W = 1280


# HELPERS

def open_webcam_imx(device_index=0):
    cap = open_webcam_imx(0)
    # cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    # if not cap.isOpened():
    #     return cap

    # Reduce buffering (prevents “fast-forward” feel due to backlog draining)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Force MJPEG (often fixes distortion/tearing and makes high-res USB cams stable)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    # Start with a stable high-res mode; move to 4K only after this is solid
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    return cap


def print_capture_info(cap):
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])
    print(f"[cam] w={w:.0f} h={h:.0f} fps={fps:.2f} fourcc={fourcc_str}")


# ----------------------------
# OCR (EasyOCR)
# ----------------------------
def init_easyocr():
    import torch
    import easyocr
    use_gpu = bool(torch.cuda.is_available())
    print(f"[EasyOCR] torch.cuda.is_available() = {use_gpu}")

    reader = easyocr.Reader(
        ['en'],
        gpu=use_gpu,
        recognizer=True,
        verbose=False
    )

    # Warmup
    try:
        _ = reader.readtext(np.zeros((80, 240, 3), dtype=np.uint8), detail=0)
    except Exception:
        pass

    return reader, use_gpu

reader, USING_GPU = init_easyocr()
os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)

detection_history = []
last_save_time = 0.0
current_clusters = []

# ----------------------------
# OCR Post-Processing (unchanged)
# ----------------------------
def fix_common_ocr_errors(text):
    text = str(text).upper().strip()
    corrections = {
        '2I1': '211', '2II': '211', '21I': '211',
        'I74': '174', 'I2': '12', 'I39': '139',
        '2OOI': '2001', '2OO5': '2005', '2OI3': '2013', '2O23': '2023',
        '744.6': '794.6', '794.G': '794.6',
        'OC': 'QC', 'CC': 'QC', '0C': 'QC',
        'O0': 'QO',
        'ENOR': 'ENGR', 'FAGR': 'ENGR', 'FNGR': 'ENGR', 'DNOIR': 'ENGR',
        'PHVS': 'PHYS',
        '@': 'Q',
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

# ----------------------------
# Helper functions (unchanged)
# ----------------------------
def crop_roi(img):
    h, w = img.shape[:2]
    x0 = int(w * ROI_X0)
    y0 = int(h * ROI_Y0)
    x1 = int(w * ROI_X1)
    y1 = int(h * ROI_Y1)
    roi = img[y0:y1, x0:x1].copy()
    return roi, (x0, y0, x1, y1)

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

def box_area(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))

def box_center(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return (sum(xs)/4, sum(ys)/4)

def box_bounds(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return min(xs), min(ys), max(xs), max(ys)

def cluster_detections_by_book(detections):
    if not detections:
        return []

    x_clusters = []
    for box, text, conf in detections:
        cx, cy = box_center(box)
        x_min, y_min, x_max, y_max = box_bounds(box)
        placed = False

        for cluster in x_clusters:
            cluster_cx = cluster["x_pos"]
            cluster_x_min = cluster["x_min"]
            cluster_x_max = cluster["x_max"]

            horizontal_overlap = not (x_max < cluster_x_min - 50 or x_min > cluster_x_max + 50)
            close_enough = abs(cx - cluster_cx) < 250

            if horizontal_overlap or close_enough:
                cluster["items"].append((box, text, conf, cx, cy))
                cluster["x_pos"] = sum(item[3] for item in cluster["items"]) / len(cluster["items"])
                cluster["x_min"] = min(cluster_x_min, x_min)
                cluster["x_max"] = max(cluster_x_max, x_max)
                cluster["all_boxes"].append(box)
                placed = True
                break

        if not placed:
            x_clusters.append({
                "x_pos": cx,
                "x_min": x_min,
                "x_max": x_max,
                "items": [(box, text, conf, cx, cy)],
                "all_boxes": [box]
            })

    x_clusters.sort(key=lambda c: c["x_pos"])
    return x_clusters

def build_call_number_from_cluster(cluster_items):
    sorted_items = sorted(cluster_items, key=lambda item: (item[4], item[3]))
    parts = []
    for box, text, conf, cx, cy in sorted_items:
        corrected = fix_common_ocr_errors(text)
        if corrected:
            parts.append(corrected)
    full_call_number = " ".join(parts)
    full_call_number = smart_prepend_single_letter(full_call_number)
    return full_call_number if full_call_number else "UNKNOWN"

def extract_core_call_number(call_num):
    normalized = normalize_call_number(call_num)
    match = re.search(r'([A-Z]{1,3}\s*\d+(?:\.\d+)?)', normalized)
    if match:
        core = match.group(1).replace(' ', ' ')
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

def update_detection_history(clusters):
    global detection_history
    now = time.time()

    for cluster in clusters:
        call_num = build_call_number_from_cluster(cluster["items"])
        conf_avg = sum(item[2] for item in cluster["items"]) / len(cluster["items"])
        if conf_avg < CONFIDENCE_THRESHOLD:
            continue

        detection_history.append({
            "call_number": call_num,
            "confidence": conf_avg,
            "text_components": [fix_common_ocr_errors(item[1]) for item in cluster["items"]],
            "timestamp": now
        })

    if len(detection_history) > 200:
        detection_history = detection_history[-200:]

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
        match = re.match(r'([A-Z]+)(\d+\.?\d*)', core)
        if match:
            return (match.group(1), float(match.group(2)))
        return (core, 0)

    try:
        stable.sort(key=sort_key)
    except Exception:
        stable.sort(key=lambda x: extract_core_call_number(x["call_number"]))

    for i, book in enumerate(stable):
        book["position"] = i + 1
        del book["detections"]

    return stable

def save_shelf_order(source_name="video_stream"):
    stable_books = get_stable_shelf_order()
    shelf_order = {"image_file": source_name, "num_books": len(stable_books), "books": stable_books}
    with open(OUTPUT_JSON, "w") as f:
        json.dump(shelf_order, f, indent=2)
    return stable_books

def easyocr_to_detections(results):
    dets = []
    for item in results:
        if item is None or len(item) < 3:
            continue
        box, text, conf = item[0], item[1], item[2]
        if text is None or len(str(text).strip()) == 0:
            continue
        box = [[float(p[0]), float(p[1])] for p in box]
        dets.append((box, str(text).strip(), float(conf) if conf is not None else 0.0))
    return dets

def process_frame(img, frame_number):
    roi, (x0, y0, x1, y1) = crop_roi(img)
    roi_proc = preprocess_for_ocr(roi)

    results = reader.readtext(
        roi_proc,
        detail=1,
        paragraph=False,
        min_size=5,
        text_threshold=0.65,
        low_text=0.4,
        link_threshold=0.4,
        canvas_size=3200,
        mag_ratio=1.5,
        slope_ths=0.3,
    )

    dets = easyocr_to_detections(results)

    new_results = []
    roi_area = roi.shape[0] * roi.shape[1]
    for box, text, conf in dets:
        area = box_area(box)
        if area < 0.0001 * roi_area or area > 0.35 * roi_area:
            continue

        x_min, y_min, x_max, y_max = box_bounds(box)
        width = x_max - x_min
        height = y_max - y_min
        if height > width * 8.0:
            continue

        new_results.append((box, text, conf))

    clusters = cluster_detections_by_book(new_results)
    return clusters, (x0, y0, x1, y1), new_results

# ----------------------------
# Main (rewritten with high-res + stable pacing + normal-duration recording)
# ----------------------------
def main():
    global last_save_time, current_clusters

    if VIDEO_PATH == "0" or VIDEO_PATH == 0:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        source_name = "webcam"

        # Reduce buffering/backlog
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Request high resolution and a sane FPS
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    else:
        if not os.path.exists(VIDEO_PATH):
            print(f"Error: Video file '{VIDEO_PATH}' not found!")
            return
        cap = cv2.VideoCapture(VIDEO_PATH)
        source_name = VIDEO_PATH

    if not cap.isOpened():
        print("Error: Could not open video source")
        return
    
    print_capture_info(cap=cap)

    # What did we actually get?
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[cam] opened={cap.isOpened()} w={actual_w:.0f} h={actual_h:.0f} fps={actual_fps:.2f}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.get(cv2.CAP_PROP_FRAME_COUNT) else 0

    # Use a fixed pacing FPS (prevents “sped up” preview and short debug videos)
    pace_fps = float(TARGET_FPS) if (VIDEO_PATH == "0" or VIDEO_PATH == 0) else float(actual_fps or 30.0)
    pace_fps = max(1.0, pace_fps)

    # Encode debug video at the same FPS we pace at
    debug_fps = int(round(pace_fps))

    print(f"Video source: {source_name}")
    print(f"Pacing FPS: {pace_fps:.1f} | Debug encode FPS: {debug_fps}")
    print(f"Output: {OUTPUT_JSON}")
    print(f"HEADLESS={HEADLESS}, SAVE_DEBUG_VIDEO={SAVE_DEBUG_VIDEO}, USING_GPU={USING_GPU}")
    print(f"OCR_HZ={OCR_HZ}")
    print("\nProcessing... (Ctrl+C to stop)\n")

    frame_count = 0
    last_save_time = time.time()
    roi_coords = None

    # Writer init later (after first disp frame)
    writer = None
    writer_inited = False

    # # ---- Metronome pacing (key fix) ----
    # period = 1.0 / pace_fps
    # t_next = time.perf_counter()

    # # OCR throttle
    # ocr_period = 1.0 / max(0.1, float(OCR_HZ))
    # next_ocr_time = time.perf_counter()
    period = 1.0 / float(TARGET_FPS)
    t_next = time.perf_counter()

    ocr_period = 1.0 / max(0.1, float(OCR_HZ))
    next_ocr_time = time.perf_counter()


# 6) At the TOP of your while True loo


    try:
        while True:
            # ---- pacing at TOP of loop ----
            # now = time.perf_counter()
            # if now < t_next:
            #     time.sleep(t_next - now)
            # t_next += period

            now = time.perf_counter()
            if now < t_next:
                time.sleep(t_next - now)
            t_next += period

            ret, frame = cap.read()
            if not ret:
                print("\nEnd of video or read error")
                break

            # Drain a couple frames so we always use latest
            for _ in range(DRAIN_FRAMES):
                ret2, frame2 = cap.read()
                if ret2:
                    frame = frame2

            frame_count += 1
            current_time = time.time()

            if frame_count == 1:
                print("[frame] shape:", frame.shape)  # (H, W, C)

            # OCR throttled by time (stable on GPU)
            if time.perf_counter() >= next_ocr_time:
                next_ocr_time = time.perf_counter() + ocr_period
                current_clusters, roi_coords, _all_detections = process_frame(frame, frame_count)

                if current_clusters:
                    update_detection_history(current_clusters)
                    print(f"Frame {frame_count}: Detected {len(current_clusters)} cluster(s)")

                if current_time - last_save_time >= UPDATE_INTERVAL:
                    stable_books = save_shelf_order(source_name)
                    print(f"\n{'='*60}")
                    print(f"Updated {OUTPUT_JSON} - {len(stable_books)} unique books")
                    for book in stable_books:
                        print(f"  {book['position']}. {book['call_number']} (×{book['detection_count']}, conf: {book['confidence']:.2f})")
                    print(f"{'='*60}\n")
                    last_save_time = current_time

            # Overlay
            if roi_coords is None:
                roi_coords = crop_roi(frame)[1]
            x0, y0, x1, y1 = roi_coords

            disp = frame.copy()
            cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 2)

            colors = [
                (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 165, 255),
                (255, 255, 0), (128, 0, 128), (255, 128, 0), (0, 255, 128),
                (128, 255, 0), (255, 0, 128)
            ]

            for idx, cluster in enumerate(current_clusters):
                color = colors[idx % len(colors)]
                for box in cluster["all_boxes"]:
                    pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
                    cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=2)

                call_num = build_call_number_from_cluster(cluster["items"])
                cx = int(cluster["x_pos"] + x0)
                cy = int(cluster["items"][0][4] + y0)

                label = f"{idx+1}: {call_num[:20]}"
                cv2.rectangle(disp, (cx - 2, cy - 16), (cx + 260, cy + 6), color, -1)
                cv2.putText(disp, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            stable_books = get_stable_shelf_order()
            cv2.putText(disp, f"Unique Books: {len(stable_books)}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            frame_text = f"Frame: {frame_count}/{total_frames if total_frames > 0 else '?'}"
            cv2.putText(disp, frame_text, (disp.shape[1] - 360, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Init writer with full-res disp size (record at full resolution)
            if SAVE_DEBUG_VIDEO and not writer_inited:
                h, w = disp.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(DEBUG_VIDEO_PATH, fourcc, debug_fps, (w, h))
                writer_inited = True
                print(f"[debug] Writing overlay video to: {DEBUG_VIDEO_PATH} @ {debug_fps} fps, size=({w}x{h})")

            if SAVE_DEBUG_VIDEO and writer is not None:
                writer.write(disp)

            # Preview (scaled down only for display)
            if not HEADLESS:
                disp_show = disp
                if disp_show.shape[1] > PREVIEW_MAX_W:
                    scale = PREVIEW_MAX_W / disp_show.shape[1]
                    disp_show = cv2.resize(disp_show, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

                cv2.imshow("Video Call Number Extraction", disp_show)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    _ = save_shelf_order(source_name)
                    print("\nForced save")

    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C).")

    finally:
        stable_books = save_shelf_order(source_name)
        print(f"\n{'='*60}")
        print("FINAL SHELF ORDER")
        print(f"{'='*60}")
        for book in stable_books:
            print(f"{book['position']}. {book['call_number']} (detected {book['detection_count']}×)")
        print(f"{'='*60}")
        print(f"Total unique books: {len(stable_books)}")
        print(f"Saved to: {OUTPUT_JSON}")

        if writer is not None:
            writer.release()
        cap.release()
        if not HEADLESS:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
