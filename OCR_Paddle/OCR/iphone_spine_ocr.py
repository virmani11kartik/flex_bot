import os
os.environ["DISABLE_MODEL_SOURCE_CHECK"] = "True"
import cv2
import numpy as np
from paddleocr import PaddleOCR
import re
import json

# Configuration
IMAGE_PATH = "sample_2.jpg"  # Change this to your image path
OUTPUT_JSON = "shelf_order.json"  # Output file for call numbers

ROI_X0, ROI_Y0 = 0.10, 0.36  # Adjust ROI
ROI_X1, ROI_Y1 = 0.95, 0.70

# ROI_X0, ROI_Y0 = 0.05, 0.50  # Adjust ROI
# ROI_X1, ROI_Y1 = 0.95, 0.92


ocr = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=False)

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

def main():
    # Load image
    if not os.path.exists(IMAGE_PATH):
        print(f"Error: Image file '{IMAGE_PATH}' not found!")
        return
    
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print(f"Error: Could not load image from '{IMAGE_PATH}'")
        return
    
    print(f"Loaded image: {IMAGE_PATH}")
    print(f"Image size: {img.shape[1]}x{img.shape[0]}")
    
    # Crop ROI
    roi, (x0, y0, x1, y1) = crop_roi(img)
    print(f"ROI size: {roi.shape[1]}x{roi.shape[0]}")
    
    # Preprocess
    print("Preprocessing image for OCR...")
    roi_proc = preprocess_for_ocr(roi)
    
    # Run OCR
    print("Running OCR...")
    result = ocr.ocr(roi_proc, cls=True)
    
    # Process results
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
    
    # Cluster by book
    clusters = cluster_detections_by_book(new_results)
    print(f"\nDetected {len(clusters)} books")
    
    # Extract call numbers and build dictionary
    shelf_order = {
        "image_file": IMAGE_PATH,
        "num_books": len(clusters),
        "books": []
    }
    
    for i, cluster in enumerate(clusters):
        call_num = build_call_number_from_cluster(cluster["items"])
        x_pos = cluster["x_pos"]
        conf_avg = sum(item[2] for item in cluster["items"]) / len(cluster["items"])
        
        book_entry = {
            "position": i + 1,
            "call_number": call_num,
            "x_position": float(x_pos),
            "confidence": float(conf_avg),
            "text_components": [item[1] for item in cluster["items"]]
        }
        
        shelf_order["books"].append(book_entry)
        
        print(f"\nBook {i+1}:")
        print(f"  Call Number: {call_num}")
        print(f"  X Position: {x_pos:.1f}")
        print(f"  Confidence: {conf_avg:.2f}")
        print(f"  Components: {book_entry['text_components']}")
    
    # Save to JSON
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(shelf_order, f, indent=2)
    
    print(f"\n{'='*60}")
    print(f"Call numbers extracted and saved to: {OUTPUT_JSON}")
    print(f"{'='*60}")
    
    # Print summary
    print("\nShelf Order (Left to Right):")
    for book in shelf_order["books"]:
        print(f"{book['position']}. {book['call_number']}")
    
    # Create visualization
    disp = img.copy()
    cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 3)
    
    colors = [(255,0,0), (0,255,255), (255,0,255), (0,165,255), (255,255,0), (128,0,128)]
    
    for idx, cluster in enumerate(clusters):
        color = colors[idx % len(colors)]
        for box in cluster["all_boxes"]:
            pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
            cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=3)
    
    y_offset = 250
    cv2.putText(disp, "Extracted Call Numbers:", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 10)
    
    for book in shelf_order["books"]:
        y_offset += 190
        display_text = f"{book['position']}. {book['call_number']}"
        cv2.putText(disp, display_text, (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 12)
    
    cv2.putText(disp, f"Books Detected: {len(clusters)}", (20, 120),
               cv2.FONT_HERSHEY_SIMPLEX, 4.0, (0, 0, 255), 8)
    
    output_path = "extracted_result.jpg"
    cv2.imwrite(output_path, disp)
    print(f"\nVisualization saved to: {output_path}")
    
    scale_percent = 30
    width = int(disp.shape[1] * scale_percent / 100)
    height = int(disp.shape[0] * scale_percent / 100)
    resized = cv2.resize(disp, (width, height), interpolation=cv2.INTER_AREA)
    
    cv2.imshow("Call Number Extraction", resized)
    print("\nPress any key to close the window...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()