import os
os.environ["DISABLE_MODEL_SOURCE_CHECK"] = "True"
import cv2
import numpy as np
from paddleocr import PaddleOCR
import json
import time
from collections import defaultdict
from difflib import SequenceMatcher

# Configuration
VIDEO_PATH = "shelf_video_2.MOV"  
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_JSON = os.path.join(BASE_DIR,"shelf_viewer", "shelf_order.json")
UPDATE_INTERVAL = 1.0  # Update JSON every 1 second

# # ROI Configuration
# ROI_X0, ROI_Y0 = 0.10, 0.36
# ROI_X1, ROI_Y1 = 0.95, 0.70

# ROI Configuration
ROI_X0, ROI_Y0 = 0.10, 0.42
ROI_X1, ROI_Y1 = 0.95, 0.77

# Detection stability settings
MIN_DETECTIONS_TO_CONFIRM = 3  # Book must be detected 3 times to be confirmed
CONFIDENCE_THRESHOLD = 0.70  # Minimum OCR confidence to accept
SIMILARITY_THRESHOLD = 0.85  # How similar call numbers must be to be same book (0-1)

ocr = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=False)

# Track detections over time for stability
detection_history = []  # List of all detected books with metadata
last_save_time = 0.0
current_clusters = []  # For visualization

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
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    gray = cv2.fastNlMeansDenoising(gray, None, 10, 7, 21)
    kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    gray = cv2.filter2D(gray, -1, kernel)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

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
    
    text_parts = []
    for box, text, conf, cx, cy in sorted_items:
        cleaned = text.strip().upper()
        if cleaned and len(cleaned) > 0:
            text_parts.append(cleaned)
    
    full_call_number = " ".join(text_parts)
    return full_call_number if full_call_number else "UNKNOWN"

def string_similarity(str1, str2):
    """Calculate similarity between two strings (0-1)"""
    return SequenceMatcher(None, str1, str2).ratio()

def normalize_call_number(call_num):
    """Normalize call number for comparison (remove extra spaces, standardize)"""
    # Remove multiple spaces
    normalized = " ".join(call_num.split())
    return normalized.upper()

def find_matching_book(call_number, existing_books):
    """Find if this call number matches an existing book"""
    normalized = normalize_call_number(call_number)
    
    for book in existing_books:
        existing_normalized = normalize_call_number(book["call_number"])
        similarity = string_similarity(normalized, existing_normalized)
        
        if similarity >= SIMILARITY_THRESHOLD:
            return book
    
    return None

def update_detection_history(clusters):
    """Add current detections to history for stability"""
    global detection_history
    
    for cluster in clusters:
        call_num = build_call_number_from_cluster(cluster["items"])
        conf_avg = sum(item[2] for item in cluster["items"]) / len(cluster["items"])
        
        # Only add if confidence is high enough
        if conf_avg < CONFIDENCE_THRESHOLD:
            continue
        
        # Create detection entry
        detection = {
            "call_number": call_num,
            "confidence": conf_avg,
            "text_components": [item[1] for item in cluster["items"]],
            "timestamp": time.time()
        }
        
        detection_history.append(detection)
    
    # Keep only recent detections (last 100)
    if len(detection_history) > 100:
        detection_history = detection_history[-100:]

def get_stable_shelf_order():
    """Get unique, stable call numbers from detection history using deduplication"""
    if not detection_history:
        return []
    
    # Group similar call numbers
    unique_books = []
    
    for detection in detection_history:
        call_num = detection["call_number"]
        
        # Try to find matching existing book
        matching_book = find_matching_book(call_num, unique_books)
        
        if matching_book:
            # Update existing book with new detection
            matching_book["detections"].append(detection)
            matching_book["detection_count"] += 1
            
            # Update average confidence
            confs = [d["confidence"] for d in matching_book["detections"]]
            matching_book["confidence"] = sum(confs) / len(confs)
            
            # Use most common call number form
            call_numbers = [d["call_number"] for d in matching_book["detections"]]
            matching_book["call_number"] = max(set(call_numbers), key=call_numbers.count)
            
            # Use most recent text components
            matching_book["text_components"] = detection["text_components"]
        else:
            # New unique book
            unique_books.append({
                "call_number": call_num,
                "confidence": detection["confidence"],
                "text_components": detection["text_components"],
                "detection_count": 1,
                "detections": [detection]
            })
    
    # Filter books that have been detected enough times
    stable_books = [b for b in unique_books if b["detection_count"] >= MIN_DETECTIONS_TO_CONFIRM]
    
    # Sort by first detection time (maintains shelf order better)
    stable_books.sort(key=lambda x: x["detections"][0]["timestamp"])
    
    # Add position numbers
    for i, book in enumerate(stable_books):
        book["position"] = i + 1
    
    # Clean up - remove detections list from output
    for book in stable_books:
        del book["detections"]
    
    return stable_books

def save_shelf_order(source_name="video_stream"):
    """Save current stable shelf order to JSON"""
    stable_books = get_stable_shelf_order()
    
    shelf_order = {
        "image_file": source_name,
        "num_books": len(stable_books),
        "books": stable_books
    }
    
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(shelf_order, f, indent=2)
    
    return stable_books

def process_frame(img, frame_number):
    """Process a single frame for OCR"""
    roi, (x0, y0, x1, y1) = crop_roi(img)
    roi_proc = preprocess_for_ocr(roi)
    
    result = ocr.ocr(roi_proc, cls=True)
    
    new_results = []
    
    if result and len(result) > 0:
        lines = result[0] if isinstance(result[0], list) or (result[0] is None) else result
        if lines is None:
            lines = []
        
        for line in lines:
            if line is None or len(line) < 2:
                continue
            
            box = line[0]
            txt = line[1]
            
            if box is None or txt is None:
                continue
            
            if not isinstance(txt, (list, tuple)) or len(txt) < 2:
                continue
            
            text, conf = txt[0], txt[1]
            
            if text is None or len(str(text).strip()) == 0:
                continue
            
            area = box_area(box)
            roi_area = roi.shape[0] * roi.shape[1]
            
            if area < 0.0003 * roi_area or area > 0.20 * roi_area:
                continue
            
            x_min, y_min, x_max, y_max = box_bounds(box)
            width = x_max - x_min
            height = y_max - y_min
            
            if height > width * 2.0:
                continue
            
            new_results.append((box, str(text).strip(), float(conf) if conf is not None else 0.0))
    
    clusters = cluster_detections_by_book(new_results)
    return clusters, (x0, y0, x1, y1), new_results

def main():
    global last_save_time, current_clusters
    
    # Open video source
    if VIDEO_PATH == "0" or VIDEO_PATH == 0:
        cap = cv2.VideoCapture(0)  # Webcam
        source_name = "webcam"
    else:
        if not os.path.exists(VIDEO_PATH):
            print(f"Error: Video file '{VIDEO_PATH}' not found!")
            return
        cap = cv2.VideoCapture(VIDEO_PATH)
        source_name = VIDEO_PATH
    
    if not cap.isOpened():
        print(f"Error: Could not open video source")
        return
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video source: {source_name}")
    print(f"FPS: {fps:.1f}")
    if total_frames > 0:
        print(f"Total frames: {total_frames}")
    print(f"Output: {OUTPUT_JSON}")
    print(f"Similarity threshold: {SIMILARITY_THRESHOLD} (books with >{SIMILARITY_THRESHOLD*100}% similar call numbers are considered duplicates)")
    print(f"\nProcessing video... Press 'q' to quit, 's' to force save\n")
    
    frame_count = 0
    process_every_n_frames = max(1, int(fps / 2))  # Process 2 frames per second
    
    last_save_time = time.time()
    roi_coords = None
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nEnd of video or read error")
                break
            
            frame_count += 1
            current_time = time.time()
            
            # Process frame periodically
            if frame_count % process_every_n_frames == 0:
                current_clusters, roi_coords, all_detections = process_frame(frame, frame_count)
                
                # Update detection history
                if current_clusters:
                    update_detection_history(current_clusters)
                    print(f"Frame {frame_count}: Detected {len(current_clusters)} cluster(s)")
                
                # Save to JSON periodically
                if current_time - last_save_time >= UPDATE_INTERVAL:
                    stable_books = save_shelf_order(source_name)
                    print(f"\n{'='*60}")
                    print(f"Updated {OUTPUT_JSON} - {len(stable_books)} unique books")
                    for book in stable_books:
                        print(f"  {book['position']}. {book['call_number']} (seen {book['detection_count']}x, conf: {book['confidence']:.2f})")
                    print(f"{'='*60}\n")
                    last_save_time = current_time
            
            # Display frame with overlay
            if roi_coords is None:
                roi_coords = crop_roi(frame)[1]
            
            x0, y0, x1, y1 = roi_coords
            disp = frame.copy()
            
            # Draw ROI
            cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 2)
            
            # Draw detection boxes with different colors per cluster
            colors = [(255,0,0), (0,255,255), (255,0,255), (0,165,255), (255,255,0), (128,0,128), 
                     (255,128,0), (0,255,128), (128,255,0), (255,0,128)]
            
            for idx, cluster in enumerate(current_clusters):
                color = colors[idx % len(colors)]
                for box in cluster["all_boxes"]:
                    pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
                    cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=2)
                
                # Draw cluster label
                call_num = build_call_number_from_cluster(cluster["items"])
                cx = int(cluster["x_pos"] + x0)
                cy = int(cluster["items"][0][4] + y0)
                
                # Draw text with background
                text = f"{idx+1}: {call_num[:20]}"  # Truncate long names
                (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                cv2.rectangle(disp, (cx-2, cy-text_height-4), (cx+text_width+2, cy+2), color, -1)
                cv2.putText(disp, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            
            # Show current stable books in top left
            stable_books = get_stable_shelf_order()
            y_offset = 30
            cv2.putText(disp, f"Unique Books: {len(stable_books)}", (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            for i, book in enumerate(stable_books[:5]):  # Show first 5
                y_offset += 30
                text = f"{book['position']}. {book['call_number'][:30]}"
                cv2.putText(disp, text, (20, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            if len(stable_books) > 5:
                y_offset += 30
                cv2.putText(disp, f"... and {len(stable_books)-5} more", (20, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.putText(disp, f"Frame: {frame_count}/{total_frames if total_frames > 0 else '?'}", 
                       (disp.shape[1] - 250, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Resize for display if too large
            if disp.shape[1] > 1280:
                scale = 1280 / disp.shape[1]
                disp = cv2.resize(disp, None, fx=scale, fy=scale)
            
            cv2.imshow("Video Call Number Extraction", disp)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                stable_books = save_shelf_order(source_name)
                print("\nForced save to JSON")
                for book in stable_books:
                    print(f"  {book['position']}. {book['call_number']}")
    
    finally:
        # Final save
        stable_books = save_shelf_order(source_name)
        print(f"\n{'='*60}")
        print("FINAL SHELF ORDER (DEDUPLICATED)")
        print(f"{'='*60}")
        for book in stable_books:
            print(f"{book['position']}. {book['call_number']} (detected {book['detection_count']} times)")
        print(f"{'='*60}")
        print(f"\nTotal unique books: {len(stable_books)}")
        print(f"Saved to: {OUTPUT_JSON}")
        
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()