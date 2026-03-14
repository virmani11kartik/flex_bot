import os
import sys
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
VIDEO_PATH = "0"  # "0" or 0 for webcam
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_JSON = os.path.join(BASE_DIR, "shelf_viewer", "shelf_order.json")
UPDATE_INTERVAL = 1.0  # seconds

# ROI Configuration (normalized)
ROI_X0, ROI_Y0 = 0.10, 0.20
ROI_X1, ROI_Y1 = 1.20, 0.90

# Detection stability settings
MIN_DETECTIONS_TO_CONFIRM = 3
CONFIDENCE_THRESHOLD = 0.75
SIMILARITY_THRESHOLD = 0.85

# Debug video
SAVE_DEBUG_VIDEO = True
DEBUG_VIDEO_PATH = os.path.join(BASE_DIR, "debug_overlay.mp4")
DEBUG_VIDEO_FPS = 30

# Display window size
DISPLAY_W, DISPLAY_H = 1280, 720

# ----------------------------
# Pygame Display
# ----------------------------
pygame_screen = None

def setup_pygame():
    """Initialize pygame for display. Returns screen surface or None."""
    global pygame_screen
    try:
        import pygame
        # Try X11 first (Jetson with monitor), fall back to offscreen
        for driver in ["x11", "fbcon", "directfb", "svgalib"]:
            os.environ["SDL_VIDEODRIVER"] = driver
            try:
                pygame.init()
                pygame_screen = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
                pygame.display.set_caption("Shelf OCR — Q to quit, S to save")
                print(f"[Display] pygame started with SDL driver: {driver}")
                return pygame_screen
            except Exception:
                pygame.quit()
                continue
        print("[Display] pygame: no working video driver found — headless mode.")
        return None
    except ImportError:
        print("[Display] pygame not installed — headless mode.")
        return None


def show_frame_pygame(screen, bgr_frame):
    """Blit a BGR OpenCV frame to the pygame window."""
    import pygame
    rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (screen.get_width(), screen.get_height()))
    # pygame expects (width, height, 3) with axes (x, y) = transpose of numpy (row, col)
    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    screen.blit(surface, (0, 0))
    pygame.display.flip()


def poll_pygame_events():
    """
    Poll pygame events.
    Returns: 'quit', 'save', or None
    """
    import pygame
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return "quit"
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                return "quit"
            if event.key == pygame.K_s:
                return "save"
    return None


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
    return reader, use_gpu


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


# ----------------------------
# Frame helpers
# ----------------------------
def crop_roi(img):
    h, w = img.shape[:2]
    x0, y0 = int(w * ROI_X0), int(h * ROI_Y0)
    x1, y1 = int(w * ROI_X1), int(h * ROI_Y1)
    return img[y0:y1, x0:x1].copy(), (x0, y0, x1, y1)


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
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def box_center(box):
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return sum(xs) / 4, sum(ys) / 4


def box_bounds(box):
    xs, ys = [p[0] for p in box], [p[1] for p in box]
    return min(xs), min(ys), max(xs), max(ys)


# def cluster_detections_by_book(detections):
#     if not detections:
#         return []
#     x_clusters = []
#     for box, text, conf in detections:
#         cx, cy = box_center(box)
#         x_min, y_min, x_max, y_max = box_bounds(box)
#         placed = False
#         for cluster in x_clusters:
#             cluster_x_min = cluster["x_min"]
#             cluster_x_max = cluster["x_max"]
#             horizontal_overlap = not (x_max < cluster_x_min - 50 or x_min > cluster_x_max + 50)
#             close_enough = abs(cx - cluster["x_pos"]) < 250
#             if horizontal_overlap or close_enough:
#                 cluster["items"].append((box, text, conf, cx, cy))
#                 cluster["x_pos"] = sum(i[3] for i in cluster["items"]) / len(cluster["items"])
#                 cluster["x_min"] = min(cluster_x_min, x_min)
#                 cluster["x_max"] = max(cluster_x_max, x_max)
#                 cluster["all_boxes"].append(box)
#                 placed = True
#                 break
#         if not placed:
#             x_clusters.append({
#                 "x_pos": cx, "x_min": x_min, "x_max": x_max,
#                 "items": [(box, text, conf, cx, cy)],
#                 "all_boxes": [box]
#             })
#     x_clusters.sort(key=lambda c: c["x_pos"])
#     return x_clusters

# def cluster_detections_by_book(detections):
#     if not detections:
#         return []

#     clusters = []

#     # Sort left → right first (important!)
#     detections = sorted(detections, key=lambda d: box_center(d[0])[0])

#     for box, text, conf in detections:
#         cx, cy = box_center(box)
#         x_min, y_min, x_max, y_max = box_bounds(box)

#         placed = False

#         for cluster in clusters:
#             cluster_x_min = cluster["x_min"]
#             cluster_x_max = cluster["x_max"]
#             cluster_width = cluster_x_max - cluster_x_min

#             # Adaptive margin (better than fixed 50px)
#             margin = max(20, int(cluster_width * 0.3))

#             horizontal_overlap = not (
#                 x_max < cluster_x_min - margin or
#                 x_min > cluster_x_max + margin
#             )

#             center_close = abs(cx - cluster["x_pos"]) < cluster_width * 0.6

#             # IMPORTANT: require BOTH
#             if horizontal_overlap and center_close:
#                 cluster["items"].append((box, text, conf, cx, cy))

#                 cluster["x_pos"] = sum(i[3] for i in cluster["items"]) / len(cluster["items"])
#                 cluster["x_min"] = min(cluster_x_min, x_min)
#                 cluster["x_max"] = max(cluster_x_max, x_max)
#                 cluster["all_boxes"].append(box)

#                 placed = True
#                 break

#         if not placed:
#             clusters.append({
#                 "x_pos": cx,
#                 "x_min": x_min,
#                 "x_max": x_max,
#                 "items": [(box, text, conf, cx, cy)],
#                 "all_boxes": [box]
#             })

#     clusters.sort(key=lambda c: c["x_pos"])
#     return clusters


def cluster_detections_by_book(detections):
    if not detections:
        return []

    # Sort detections left → right FIRST (very important)
    detections = sorted(detections, key=lambda d: box_center(d[0])[0])

    clusters = []

    for box, text, conf in detections:
        cx, cy = box_center(box)
        x_min, y_min, x_max, y_max = box_bounds(box)

        placed = False

        for cluster in clusters:
            cluster_width = cluster["x_max"] - cluster["x_min"]

            # Adaptive margin instead of fixed 50px
            margin = max(15, int(cluster_width * 0.25))

            horizontal_overlap = not (
                x_max < cluster["x_min"] - margin or
                x_min > cluster["x_max"] + margin
            )

            center_close = abs(cx - cluster["x_pos"]) < cluster_width * 0.6

            # IMPORTANT: require BOTH conditions
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


def process_frame(img, reader):
    roi, coords = crop_roi(img)
    roi_proc = preprocess_for_ocr(roi)
    results = reader.readtext(
        roi_proc, detail=1, paragraph=False,
        min_size=5, text_threshold=0.65, low_text=0.4,
        link_threshold=0.4, canvas_size=3200, mag_ratio=1.5, slope_ths=0.3,
    )
    dets = easyocr_to_detections(results)
    roi_area = roi.shape[0] * roi.shape[1]
    filtered = []
    for box, text, conf in dets:
        area = box_area(box)
        if area < 0.0001 * roi_area or area > 0.35 * roi_area:
            continue
        x_min, y_min, x_max, y_max = box_bounds(box)
        if (y_max - y_min) > (x_max - x_min) * 8.0:
            continue
        filtered.append((box, text, conf))
    return cluster_detections_by_book(filtered), coords, filtered


# ----------------------------
# Detection history
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


def save_shelf_order(source_name, output_json):
    stable_books = get_stable_shelf_order()
    shelf_order = {
        "image_file": source_name,
        "num_books": len(stable_books),
        "books": stable_books
    }
    with open(output_json, "w") as f:
        json.dump(shelf_order, f, indent=2)
    return stable_books


# ----------------------------
# Overlay drawing
# ----------------------------
COLORS = [
    (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 165, 255),
    (255, 255, 0), (128, 0, 128), (255, 128, 0), (0, 255, 128),
    (128, 255, 0), (255, 0, 128)
]

def draw_overlay(frame, roi_coords, clusters, stable_books, frame_count, total_frames, headless):
    x0, y0, x1, y1 = roi_coords
    disp = frame.copy()

    # ROI box
    cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 2)

    # Cluster boxes + labels
    for idx, cluster in enumerate(clusters):
        color = COLORS[idx % len(COLORS)]
        for box in cluster["all_boxes"]:
            pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
            cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=2)
        call_num = build_call_number_from_cluster(cluster["items"])
        cx = int(cluster["x_pos"] + x0)
        cy = int(cluster["items"][0][4] + y0)
        label = f"{idx + 1}: {call_num[:20]}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        cv2.rectangle(disp, (cx - 2, cy - th - 4), (cx + tw + 2, cy + 2), color, -1)
        cv2.putText(disp, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Book list
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

    # Mode + frame counter
    mode = "HEADLESS" if headless else "LIVE (pygame)"
    cv2.putText(disp, mode, (20, disp.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
    cv2.putText(disp, f"Frame: {frame_count}/{total_frames or '?'}",
                (disp.shape[1] - 280, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if disp.shape[1] > 1280:
        scale = 1280 / disp.shape[1]
        disp = cv2.resize(disp, None, fx=scale, fy=scale)

    return disp


# ----------------------------
# Main
# ----------------------------
def main():
    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)

    # Init display
    screen = setup_pygame()
    headless = screen is None

    # Init OCR
    reader, using_gpu = init_easyocr()

    # Open video
    if VIDEO_PATH in ("0", 0):
        cap = cv2.VideoCapture(0)
        source_name = "webcam"
    else:
        if not os.path.exists(VIDEO_PATH):
            print(f"Error: '{VIDEO_PATH}' not found!")
            return
        cap = cv2.VideoCapture(VIDEO_PATH)
        source_name = VIDEO_PATH

    if not cap.isOpened():
        print("Error: Could not open video source")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0

    print(f"\nSource     : {source_name}")
    print(f"FPS        : {fps:.1f}")
    print(f"GPU        : {using_gpu}")
    print(f"Display    : {'HEADLESS' if headless else 'pygame window'}")
    print(f"Debug video: {DEBUG_VIDEO_PATH if SAVE_DEBUG_VIDEO else 'off'}")
    print(f"Output JSON: {OUTPUT_JSON}")
    print("\nRunning... (Ctrl+C or Q in window to stop)\n")

    frame_count = 0
    process_every_n = max(1, int(fps / 2))
    last_save_time = time.time()
    current_clusters = []
    roi_coords = None
    writer = None
    writer_inited = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nEnd of stream.")
                break

            frame_count += 1
            now = time.time()

            # OCR every N frames
            if frame_count % process_every_n == 0:
                current_clusters, roi_coords, _ = process_frame(frame, reader)
                if current_clusters:
                    update_detection_history(current_clusters)
                    print(f"Frame {frame_count}: {len(current_clusters)} cluster(s)")

                if now - last_save_time >= UPDATE_INTERVAL:
                    stable = save_shelf_order(source_name, OUTPUT_JSON)
                    print(f"\n{'='*55}")
                    print(f"Saved {len(stable)} books → {OUTPUT_JSON}")
                    for b in stable:
                        print(f"  {b['position']}. {b['call_number']} "
                              f"(×{b['detection_count']}, conf {b['confidence']:.2f})")
                    print(f"{'='*55}\n")
                    last_save_time = now

            if roi_coords is None:
                roi_coords = crop_roi(frame)[1]

            stable_books = get_stable_shelf_order()
            disp = draw_overlay(frame, roi_coords, current_clusters,
                                stable_books, frame_count, total_frames, headless)

            # Init video writer
            if SAVE_DEBUG_VIDEO and not writer_inited:
                h, w = disp.shape[:2]
                writer = cv2.VideoWriter(
                    DEBUG_VIDEO_PATH,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    DEBUG_VIDEO_FPS, (w, h)
                )
                writer_inited = True
                print(f"[Debug video] → {DEBUG_VIDEO_PATH}")

            if SAVE_DEBUG_VIDEO and writer:
                writer.write(disp)

            # Display
            if not headless and screen:
                show_frame_pygame(screen, disp)
                action = poll_pygame_events()
                if action == "quit":
                    print("Quit.")
                    break
                elif action == "save":
                    save_shelf_order(source_name, OUTPUT_JSON)
                    print("Forced save.")

    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C).")

    finally:
        stable = save_shelf_order(source_name, OUTPUT_JSON)
        print(f"\n{'='*55}")
        print("FINAL SHELF ORDER")
        print(f"{'='*55}")
        for b in stable:
            print(f"  {b['position']}. {b['call_number']} (×{b['detection_count']})")
        print(f"Total: {len(stable)} books")
        print(f"JSON : {OUTPUT_JSON}")
        print(f"{'='*55}")

        if writer:
            writer.release()
        cap.release()

        try:
            import pygame
            pygame.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
