import os
import cv2
import json
import time
import numpy as np
import re
from collections import defaultdict
from difflib import SequenceMatcher

# ----------------------------
# Configuration
# ----------------------------
VIDEO_PATH = "shelf_video_2.MOV"  # "0" or 0 for webcam
VIDEO_PATH = "0"  # "0" or 0 for webcam
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_JSON = os.path.join(BASE_DIR, "shelf_viewer", "shelf_order.json")
UPDATE_INTERVAL = 1.0  # seconds

# ROI Configuration (normalized)
ROI_X0, ROI_Y0 = 0.10, 0.42
ROI_X1, ROI_Y1 = 0.95, 0.77

# Detection stability settings
MIN_DETECTIONS_TO_CONFIRM = 3
CONFIDENCE_THRESHOLD = 0.75  # Lowered to catch more detections
SIMILARITY_THRESHOLD = 0.85

# Headless / display handling
HEADLESS = True
SAVE_DEBUG_VIDEO = True
DEBUG_VIDEO_PATH = os.path.join(BASE_DIR, "debug_overlay.mp4")
DEBUG_VIDEO_FPS = 30

# ----------------------------
# OCR (EasyOCR)
# ----------------------------
def init_easyocr():
    """Initialize EasyOCR with GPU support"""
    import torch
    import easyocr

    use_gpu = bool(torch.cuda.is_available())
    print(f"[EasyOCR] torch.cuda.is_available() = {use_gpu}")
    
    # EasyOCR with optimized settings for book spines
    reader = easyocr.Reader(
        ['en'], 
        gpu=use_gpu,
        recognizer=True,
        verbose=False
    )
    return reader, use_gpu

reader, USING_GPU = init_easyocr()

# Ensure output dir exists
os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)

# Track detections over time
detection_history = []
last_save_time = 0.0
current_clusters = []

# ----------------------------
# OCR Post-Processing
# ----------------------------
def fix_common_ocr_errors(text):
    """Fix common OCR misreads in library call numbers"""
    text = str(text).upper().strip()
    
    # Common whole-word corrections
    corrections = {
        '2I1': '211',
        '2II': '211', 
        '21I': '211',
        'I74': '174',
        'I2': '12',
        'I39': '139',
        '2OOI': '2001',
        '2OO5': '2005',
        '2OI3': '2013',
        '2O23': '2023',
        '744.6': '794.6',
        '794.G': '794.6',
        'OC': 'QC',
        'CC': 'QC',
        '0C': 'QC',
        'O0': 'QO',
        'ENOR': 'ENGR',
        'FAGR': 'ENGR',
        'FNGR': 'ENGR',
        'DNOIR': 'ENGR',
        'PHVS': 'PHYS',
        '@': 'Q',
    }
    
    # Apply direct replacements
    for wrong, right in corrections.items():
        text = text.replace(wrong, right)
    
    # Fix I/1 confusion
    # "I" before or after digits should be "1"
    text = re.sub(r'I(?=\d)', '1', text)  # I before digit
    text = re.sub(r'(?<=\d)I(?=\d)', '1', text)  # I between digits
    text = re.sub(r'(?<=\d)I\b', '1', text)  # I after digit at word boundary
    
    # Fix O/0 confusion in numbers
    text = re.sub(r'(?<=\d)O(?=\d)', '0', text)  # O between digits
    text = re.sub(r'(?<=\d)O\b', '0', text)  # O at end after digit
    text = re.sub(r'\b2O', '20', text)  # Common: 2001, 2005, etc.
    
    # Fix l/1 confusion
    text = re.sub(r'l(?=\d)', '1', text)
    text = re.sub(r'(?<=\d)l', '1', text)
    
    # Fix G/6 confusion in numbers
    text = re.sub(r'(?<=\d)G\b', '6', text)
    text = re.sub(r'\.G\b', '.6', text)
    
    return text

def smart_prepend_single_letter(call_number):
    """
    Detect if call number is missing a single letter prefix.
    LC call numbers start with 1-3 letters. If we see a number at the start,
    we might be missing a letter (especially Q which is small and hard to read).
    """
    normalized = normalize_call_number(call_number)
    
    # Check if starts with digit (missing letter class)
    if re.match(r'^\d', normalized):
        # Check the number - certain ranges are associated with Q
        match = re.match(r'^(\d+)', normalized)
        if match:
            num = int(match.group(1))
            # Q classification ranges (common ones):
            # Q 300-390 (Cybernetics, AI, etc.)
            # Q 325 specifically often appears
            if 300 <= num <= 399:
                return 'Q ' + normalized
    
    return call_number

# ----------------------------
# Helper functions
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
    """Enhanced preprocessing with multiple strategies for better OCR"""
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    
    # Strategy 1: Sharpen first to enhance edges
    kernel_sharpen = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    sharpened = cv2.filter2D(gray, -1, kernel_sharpen)
    
    # Strategy 2: CLAHE for contrast
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(sharpened)
    
    # Strategy 3: Bilateral filter to reduce noise while keeping edges
    bilateral = cv2.bilateralFilter(enhanced, 5, 75, 75)
    
    # Strategy 4: Morphological operations to enhance text
    kernel_morph = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    morph = cv2.morphologyEx(bilateral, cv2.MORPH_CLOSE, kernel_morph)
    
    # Return RGB for EasyOCR
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
    """Build call number with OCR error correction"""
    sorted_items = sorted(cluster_items, key=lambda item: (item[4], item[3]))
    parts = []
    
    for box, text, conf, cx, cy in sorted_items:
        # Apply OCR corrections
        corrected = fix_common_ocr_errors(text)
        if corrected:
            parts.append(corrected)
    
    full_call_number = " ".join(parts)
    
    # Apply smart letter prepending for missing Q, etc.
    full_call_number = smart_prepend_single_letter(full_call_number)
    
    return full_call_number if full_call_number else "UNKNOWN"

def string_similarity(str1, str2):
    return SequenceMatcher(None, str1, str2).ratio()

def normalize_call_number(call_num):
    """Normalize call number - remove prefixes"""
    normalized = " ".join(str(call_num).split()).upper()
    
    # Remove common prefix words
    prefixes = ['ENGR', 'MATH', 'PHYS', 'DNOIR', 'ENOR', 'FAGR', 'FNGR', 'SCI', 'TECH']
    words = normalized.split()
    
    while words and words[0] in prefixes:
        words.pop(0)
    
    return " ".join(words) if words else normalized

def extract_core_call_number(call_num):
    """Extract core LC classification (CLASS + NUMBER)"""
    normalized = normalize_call_number(call_num)
    
    # Pattern: 1-3 letters followed by numbers
    match = re.search(r'([A-Z]{1,3}\s*\d+(?:\.\d+)?)', normalized)
    if match:
        core = match.group(1).replace(' ', ' ')
        # Include cutter if present
        cutter_match = re.search(r'([A-Z]\d+)', normalized[match.end():])
        if cutter_match:
            core += ' ' + cutter_match.group(1)
        return core
    
    return normalized

def books_are_same(call_num1, call_num2):
    """Check if two call numbers are the same book"""
    core1 = extract_core_call_number(call_num1)
    core2 = extract_core_call_number(call_num2)
    
    # Exact core match
    if core1 == core2 and core1:
        return True
    
    # String similarity
    norm1 = normalize_call_number(call_num1)
    norm2 = normalize_call_number(call_num2)
    
    if string_similarity(norm1, norm2) >= SIMILARITY_THRESHOLD:
        return True
    
    # Substring matching for partial detections
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
    """Get stable books with smart deduplication"""
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
            
            # Use longest/most complete call number
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
    
    # Sort by LC classification
    def sort_key(book):
        core = extract_core_call_number(book["call_number"])
        match = re.match(r'([A-Z]+)(\d+\.?\d*)', core)
        if match:
            return (match.group(1), float(match.group(2)))
        return (core, 0)
    
    try:
        stable.sort(key=sort_key)
    except:
        stable.sort(key=lambda x: extract_core_call_number(x["call_number"]))

    for i, book in enumerate(stable):
        book["position"] = i + 1
        del book["detections"]

    return stable

def save_shelf_order(source_name="video_stream"):
    stable_books = get_stable_shelf_order()
    shelf_order = {
        "image_file": source_name,
        "num_books": len(stable_books),
        "books": stable_books
    }
    with open(OUTPUT_JSON, "w") as f:
        json.dump(shelf_order, f, indent=2)
    return stable_books

def easyocr_to_detections(results):
    """
    EasyOCR results: list of [box, text, conf]
    """
    dets = []
    for item in results:
        if item is None or len(item) < 3:
            continue
        box, text, conf = item[0], item[1], item[2]
        if text is None or len(str(text).strip()) == 0:
            continue
        
        # Normalize box
        box = [[float(p[0]), float(p[1])] for p in box]
        dets.append((box, str(text).strip(), float(conf) if conf is not None else 0.0))
    return dets

def process_frame(img, frame_number):
    roi, (x0, y0, x1, y1) = crop_roi(img)
    roi_proc = preprocess_for_ocr(roi)

    # EasyOCR with better parameters
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
    
    new_results = []
    dets = easyocr_to_detections(results)

    roi_area = roi.shape[0] * roi.shape[1]
    for box, text, conf in dets:
        area = box_area(box)
        
        # Adjusted filtering - less aggressive
        if area < 0.0001 * roi_area or area > 0.35 * roi_area:
            continue

        x_min, y_min, x_max, y_max = box_bounds(box)
        width = x_max - x_min
        height = y_max - y_min

        # Allow taller boxes (book spines)
        if height > width * 8.0:
            continue

        new_results.append((box, text, conf))

    clusters = cluster_detections_by_book(new_results)
    return clusters, (x0, y0, x1, y1), new_results

# ----------------------------
# Main
# ----------------------------
def main():
    global last_save_time, current_clusters

    if VIDEO_PATH == "0" or VIDEO_PATH == 0:
        cap = cv2.VideoCapture(0)
        source_name = "webcam"
    else:
        if not os.path.exists(VIDEO_PATH):
            print(f"Error: Video file '{VIDEO_PATH}' not found!")
            return
        cap = cv2.VideoCapture(VIDEO_PATH)
        source_name = VIDEO_PATH

    if not cap.isOpened():
        print("Error: Could not open video source")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.get(cv2.CAP_PROP_FRAME_COUNT) else 0

    print(f"Video source: {source_name}")
    print(f"FPS: {fps:.1f}")
    if total_frames > 0:
        print(f"Total frames: {total_frames}")
    print(f"Output: {OUTPUT_JSON}")
    print(f"OCR Error Correction: ENABLED")
    print(f"HEADLESS={HEADLESS}, SAVE_DEBUG_VIDEO={SAVE_DEBUG_VIDEO}, USING_GPU={USING_GPU}")
    print("\nProcessing video... (Ctrl+C to stop)\n")

    frame_count = 0
    process_every_n_frames = max(1, int(fps / 2))
    last_save_time = time.time()
    roi_coords = None

    writer = None
    writer_inited = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nEnd of video or read error")
                break

            frame_count += 1
            current_time = time.time()

            if frame_count % process_every_n_frames == 0:
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

            # Draw overlays
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
                (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                cv2.rectangle(disp, (cx - 2, cy - th - 4), (cx + tw + 2, cy + 2), color, -1)
                cv2.putText(disp, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            stable_books = get_stable_shelf_order()
            y_offset = 30
            cv2.putText(disp, f"Unique Books: {len(stable_books)}", (20, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            for i, book in enumerate(stable_books[:5]):
                y_offset += 30
                txt = f"{book['position']}. {book['call_number'][:30]}"
                cv2.putText(disp, txt, (20, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if len(stable_books) > 5:
                y_offset += 30
                cv2.putText(disp, f"... and {len(stable_books)-5} more", (20, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            frame_text = f"Frame: {frame_count}/{total_frames if total_frames > 0 else '?'}"
            cv2.putText(disp, frame_text, (disp.shape[1] - 300, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            if disp.shape[1] > 1280:
                scale = 1280 / disp.shape[1]
                disp = cv2.resize(disp, None, fx=scale, fy=scale)

            if SAVE_DEBUG_VIDEO and not writer_inited:
                h, w = disp.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(DEBUG_VIDEO_PATH, fourcc, DEBUG_VIDEO_FPS, (w, h))
                writer_inited = True
                print(f"[debug] Writing overlay video to: {DEBUG_VIDEO_PATH}")

            if SAVE_DEBUG_VIDEO and writer is not None:
                writer.write(disp)

            if not HEADLESS:
                cv2.imshow("Video Call Number Extraction", disp)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    stable_books = save_shelf_order(source_name)
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
