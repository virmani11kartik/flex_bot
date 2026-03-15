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

CAMERA_ID = 4

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

# Detection stability settings (from code 1)
MIN_DETECTIONS_TO_CONFIRM = 3
CONFIDENCE_THRESHOLD = 0.75
SIMILARITY_THRESHOLD = 0.85

COLORS = [
    (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 165, 255),
    (255, 255, 0), (128, 0, 128), (255, 128, 0), (0, 255, 128),
    (128, 255, 0), (255, 0, 128)
]


# ----------------------------
# Rectangle Detection
# ----------------------------

def detect_label_rectangle(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,180])
    upper_white = np.array([180,50,255])

    mask = cv2.inRange(hsv,lower_white,upper_white)

    kernel = np.ones((3,3),np.uint8)
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    best_rect=None
    best_area=0

    for contour in contours:

        area=cv2.contourArea(contour)

        if area>500 and area<50000:

            peri=cv2.arcLength(contour,True)
            approx=cv2.approxPolyDP(contour,0.02*peri,True)

            if len(approx)==4 and cv2.isContourConvex(approx):

                x,y,w,h=cv2.boundingRect(approx)

                if (MIN_WIDTH<=w<=MAX_WIDTH) and (MIN_HEIGHT<=h<=MAX_HEIGHT):

                    if area>best_area:
                        best_area=area
                        best_rect=(x,y,w,h)

    if best_rect is None:
        return None

    x,y,w,h=best_rect
    cx=x+w//2
    cy=y+h//2

    return cx,cy,w,h


# ----------------------------
# OCR INIT
# ----------------------------

def init_easyocr():

    import torch
    import easyocr

    use_gpu=torch.cuda.is_available()

    reader=easyocr.Reader(
        ['en'],
        gpu=use_gpu,
        recognizer=True,
        verbose=False
    )

    return reader


# ----------------------------
# OCR Post-Processing (from code 1)
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

            horizontal_overlap = not (
                x_max < cluster["x_min"] - margin or
                x_min > cluster["x_max"] + margin
            )
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
                "x_pos": cx,
                "x_min": x_min,
                "x_max": x_max,
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
# OCR PREPROCESS (from code 1)
# ----------------------------

def preprocess_for_ocr(roi_bgr):

    gray=cv2.cvtColor(roi_bgr,cv2.COLOR_BGR2GRAY)

    kernel_sharpen=np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
    sharpened=cv2.filter2D(gray,-1,kernel_sharpen)

    clahe=cv2.createCLAHE(clipLimit=3.0,tileGridSize=(8,8))
    enhanced=clahe.apply(sharpened)

    bilateral=cv2.bilateralFilter(enhanced,5,75,75)

    kernel_morph=cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))
    morph=cv2.morphologyEx(bilateral,cv2.MORPH_CLOSE,kernel_morph)

    return cv2.cvtColor(morph,cv2.COLOR_GRAY2BGR)


# ----------------------------
# OCR RUN (from code 1)
# ----------------------------

def run_ocr(frame, reader, rect):
    cx, cy, rw, rh = rect

    padding = 10
    x0 = max(0, cx - rw // 2 - padding)
    y0 = max(0, cy - rh // 2 - padding)
    x1 = min(frame.shape[1], cx + rw // 2 + padding)
    y1 = min(frame.shape[0], cy + rh // 2 + padding)

    rect_roi = frame[y0:y1, x0:x1].copy()

    roi_proc = preprocess_for_ocr(rect_roi)

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

    # Adjust box coords back to full frame space
    adjusted_dets = []
    for box, text, conf in dets:
        adjusted_box = [[p[0] + x0, p[1] + y0] for p in box]
        adjusted_dets.append((adjusted_box, text, conf))

    roi_area = rect_roi.shape[0] * rect_roi.shape[1]
    filtered = []

    for box, text, conf in adjusted_dets:
        area = box_area(box)
        if area < 0.0001 * roi_area or area > 0.35 * roi_area:
            continue
        x_min, y_min, x_max, y_max = box_bounds(box)
        if (y_max - y_min) > (x_max - x_min) * 8.0:
            continue
        filtered.append((box, text, conf))

    clusters = cluster_detections_by_book(filtered)
    return clusters


# ----------------------------
# Detection history (from code 1)
# ----------------------------

detection_history = []

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
    shelf_order = {
        "num_books": len(stable_books),
        "books": stable_books
    }
    with open(output_json, "w") as f:
        json.dump(shelf_order, f, indent=2)
    return stable_books


# ----------------------------
# DRAW OVERLAY (from code 1)
# ----------------------------

def draw_overlay(frame, rect, aligned, clusters, stable_books, roi_coords, frame_count):

    disp = frame.copy()

    x0, y0, x1, y1 = roi_coords

    # ROI box
    cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 2)

    # ROI center line
    roi_center_y = (y0 + y1) // 2
    cv2.line(disp, (x0, roi_center_y), (x1, roi_center_y), (0, 255, 255), 1)

    # Detected label rectangle
    if rect is not None:
        cx, cy, w, h = rect
        cv2.rectangle(
            disp,
            (cx - w // 2, cy - h // 2),
            (cx + w // 2, cy + h // 2),
            (0, 0, 255),
            2
        )
        cv2.circle(disp, (cx, cy), 5, (255, 0, 0), -1)

        status = "ALIGNED" if aligned else "MOVING"
        color = (0, 255, 0) if aligned else (0, 165, 255)
        cv2.putText(disp, status, (cx - 40, cy - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Cluster boxes + call number labels (boxes already in full frame coords)
    for idx, cluster in enumerate(clusters):
        color = COLORS[idx % len(COLORS)]
        for box in cluster["all_boxes"]:
            pts = np.array([[int(p[0]), int(p[1])] for p in box], dtype=np.int32)
            cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=2)
        call_num = build_call_number_from_cluster(cluster["items"])
        cx_c = int(cluster["x_pos"])
        cy_c = int(cluster["items"][0][4])
        label = f"{idx + 1}: {call_num[:20]}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        cv2.rectangle(disp, (cx_c - 2, cy_c - th - 4), (cx_c + tw + 2, cy_c + 2), color, -1)
        cv2.putText(disp, label, (cx_c, cy_c), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Stable book list
    y_off = 30
    cv2.putText(disp, f"Unique Books: {len(stable_books)}", (20, y_off),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    for book in stable_books[:5]:
        y_off += 30
        cv2.putText(disp, f"{book['position']}. {book['call_number'][:30]}", (20, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    if len(stable_books) > 5:
        y_off += 30
        cv2.putText(disp, f"... and {len(stable_books) - 5} more", (20, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.putText(disp, f"Frame: {frame_count}",
                (disp.shape[1] - 200, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return disp


# Add this function to check focus/quality
def is_frame_quality_good(frame, rect):
    if rect is None:
        return False
    
    cx, cy, w, h = rect
    # Extract the rectangle region
    x0 = max(0, cx - w//2 - 10)
    y0 = max(0, cy - h//2 - 10)
    x1 = min(frame.shape[1], cx + w//2 + 10)
    y1 = min(frame.shape[0], cy + h//2 + 10)
    
    roi = frame[y0:y1, x0:x1]
    
    # Method 1: Laplacian variance (higher = sharper)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    
    # Method 2: Check if rectangle detection is stable for X frames
    # (you'd need to track this across frames)
    
    return laplacian_var > 100  # Adjust threshold as needed
# ----------------------------
# MAIN
# ----------------------------

def main():

    os.makedirs(os.path.dirname(OUTPUT_JSON),exist_ok=True)

    reader=init_easyocr()

    cap=cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        print("Camera open failed")
        return

    current_clusters = []
    frame_count=0
    last_save=time.time()

    while True:

        ret,frame=cap.read()

        if not ret:
            break

        frame=cv2.resize(frame,(DISPLAY_W,DISPLAY_H))

        frame_count+=1

        h,w=frame.shape[:2]

        roi_x0=int(w*0.10)
        roi_y0=int(h*0.20)
        roi_x1=int(w*0.90)
        roi_y1=int(h*0.90)

        roi_coords=(roi_x0,roi_y0,roi_x1,roi_y1)

        roi_center_y=(roi_y0+roi_y1)//2

        rect=detect_label_rectangle(frame)

        aligned=False
        target_center_x=0
        target_center_y=0
        target_width=0
        target_height=0
        alignment_error=999

        if rect is not None:

            cx,cy,w_rect,h_rect=rect

            target_center_x=cx
            target_center_y=cy
            target_width=w_rect
            target_height=h_rect

            alignment_error=abs(cy-roi_center_y)

            aligned=alignment_error<ALIGNMENT_TOLERATION

            if aligned:

                if is_frame_quality_good(frame, rect):
                    current_clusters = run_ocr(frame, reader, rect)
                    if current_clusters:
                        update_detection_history(current_clusters)
                else:
                    print("Waiting for focus...")

                if time.time()-last_save>UPDATE_INTERVAL:

                    stable = save_shelf_order(OUTPUT_JSON)
                    print(f"Saved {len(stable)} books → {OUTPUT_JSON}")
                    for b in stable:
                        print(f"  {b['position']}. {b['call_number']} "
                              f"(x{b['detection_count']}, conf {b['confidence']:.2f})")
                    last_save=time.time()

        state={
            "clusters":[],
            "roi_height":roi_coords[3]-roi_coords[1],
            "roi_center_x":(roi_coords[0]+roi_coords[2])//2,
            "roi_center_y":roi_center_y,
            "has_target":rect is not None,
            "target_center_x":target_center_x,
            "target_center_y":target_center_y,
            "target_width":target_width,
            "target_height":target_height,
            "alignment_error":alignment_error,
            "is_aligned":aligned,
            "timestamp":time.time()
        }

        with open(os.path.expanduser("~/Desktop/autonomous_state.json"),"w") as f:
            json.dump(state,f)

        stable_books = get_stable_shelf_order()
        disp=draw_overlay(frame,rect,aligned,current_clusters,stable_books,roi_coords,frame_count)

        cv2.imshow("Shelf OCR",disp)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__=="__main__":
    main()