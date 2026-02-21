import os
os.environ["DISABLE_MODEL_SOURCE_CHECK"] = "True"
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from paddleocr import PaddleOCR
import re

# RealSense Configuration - OPTIMIZED FOR IMAGE QUALITY
COLOR_W, COLOR_H, FPS = 1920, 1080, 15  # Higher resolution, lower FPS for better quality

# ROI Configuration
ROI_X0, ROI_Y0 = 0.25, 0.50
ROI_X1, ROI_Y1 = 0.80, 0.75

# OCR Configuration
OCR_PERIOD_S = 1.0  # Run OCR every 1 second (slower for stability)

# Multi-frame averaging for better image quality
FRAME_BUFFER_SIZE = 5  # Average 5 frames to reduce noise

ocr = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=False)

# Initialize RealSense pipeline with optimized settings
pipeline = rs.pipeline()
config = rs.config()

# Try to find and configure the device
ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    raise RuntimeError("No RealSense device connected!")

print(f"Found RealSense device: {devices[0].get_info(rs.camera_info.name)}")

# Enable color stream with fallback options
try:
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    profile = pipeline.start(config)
    print(f"Started stream: {COLOR_W}x{COLOR_H} @ {FPS}fps")
except RuntimeError as e:
    print(f"Failed to start with {COLOR_W}x{COLOR_H} @ {FPS}fps")
    print("Trying fallback configuration: 1280x720 @ 30fps")
    config = rs.config()
    COLOR_W, COLOR_H, FPS = 1280, 720, 30
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    profile = pipeline.start(config)

# Get the color sensor and adjust camera settings
try:
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    print("\nConfiguring camera settings...")
    
    # Disable auto-exposure initially to set manual values
    if color_sensor.supports(rs.option.enable_auto_exposure):
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        print("  Auto-exposure: ENABLED")
        
    # Adjust exposure (if supported)
    if color_sensor.supports(rs.option.exposure):
        try:
            color_sensor.set_option(rs.option.exposure, 166)
            print(f"  Exposure: {color_sensor.get_option(rs.option.exposure)}")
        except:
            print("  Exposure: Could not set")
    
    # Adjust gain (brightness)
    if color_sensor.supports(rs.option.gain):
        try:
            color_sensor.set_option(rs.option.gain, 64)
            print(f"  Gain: {color_sensor.get_option(rs.option.gain)}")
        except:
            print("  Gain: Could not set")
    
    # Adjust white balance
    if color_sensor.supports(rs.option.enable_auto_white_balance):
        color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
        print("  Auto white balance: ENABLED")
    
    # Adjust sharpness
    if color_sensor.supports(rs.option.sharpness):
        try:
            color_sensor.set_option(rs.option.sharpness, 50)
            print(f"  Sharpness: {color_sensor.get_option(rs.option.sharpness)}")
        except:
            print("  Sharpness: Could not set")
    
    # Adjust saturation
    if color_sensor.supports(rs.option.saturation):
        try:
            color_sensor.set_option(rs.option.saturation, 64)
            print(f"  Saturation: {color_sensor.get_option(rs.option.saturation)}")
        except:
            print("  Saturation: Could not set")
    
    # Adjust contrast
    if color_sensor.supports(rs.option.contrast):
        try:
            color_sensor.set_option(rs.option.contrast, 50)
            print(f"  Contrast: {color_sensor.get_option(rs.option.contrast)}")
        except:
            print("  Contrast: Could not set")
    
except Exception as e:
    print(f"Warning: Could not configure all camera settings: {e}")
    print("Continuing with default settings...")
    color_sensor = None

# Warm up camera
print("Warming up camera (30 frames)...")
for i in range(30):
    pipeline.wait_for_frames()
    if i % 10 == 0:
        print(f"  {i}/30...")

print("Camera ready!\n")

# Frame buffer for averaging
frame_buffer = []

last_ocr_t = 0.0
cached_results = []
cached_clusters = []
cached_call_numbers = []

t0 = time.time()
frame_count = 0
disp_fps = 0

def crop_roi(img):
    h, w = img.shape[:2]
    x0 = int(w * ROI_X0)
    y0 = int(h * ROI_Y0)
    x1 = int(w * ROI_X1)
    y1 = int(h * ROI_Y1)
    roi = img[y0:y1, x0:x1].copy()
    return roi, (x0, y0, x1, y1)

def average_frames(buffer):
    """Average multiple frames to reduce noise"""
    if len(buffer) == 0:
        return None
    return np.mean(buffer, axis=0).astype(np.uint8)

def preprocess_for_ocr(roi_bgr):
    """Enhanced preprocessing for better OCR results on D455"""
    # Convert to grayscale
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    
    # Apply bilateral filter to reduce noise while preserving edges
    gray = cv2.bilateralFilter(gray, 9, 75, 75)
    
    # Apply CLAHE for better contrast
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    
    # Advanced denoising
    gray = cv2.fastNlMeansDenoising(gray, None, h=10, templateWindowSize=7, searchWindowSize=21)
    
    # Sharpen using unsharp mask
    gaussian = cv2.GaussianBlur(gray, (0, 0), 2.0)
    gray = cv2.addWeighted(gray, 2.0, gaussian, -1.0, 0)
    
    # Morphological operations to enhance text
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    
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

try:
    print("\nStarting OPTIMIZED RealSense D455 Book Shelf Reading...")
    print("=" * 60)
    print("Controls:")
    print("  'q' or ESC - Quit")
    print("  's' - Save current shelf order")
    print("  'c' - Capture current frame")
    print("  'e' - Adjust exposure (+10)")
    print("  'E' - Adjust exposure (-10)")
    print("  'g' - Adjust gain (+5)")
    print("  'G' - Adjust gain (-5)")
    print("=" * 60 + "\n")
    
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        
        # Add to frame buffer for averaging
        frame_buffer.append(img.copy())
        if len(frame_buffer) > FRAME_BUFFER_SIZE:
            frame_buffer.pop(0)
        
        now = time.time()
        
        # Run OCR periodically
        if (now - last_ocr_t) >= OCR_PERIOD_S and len(frame_buffer) == FRAME_BUFFER_SIZE:
            print(f"[{time.strftime('%H:%M:%S')}] Running OCR on averaged frame...")
            
            # Use averaged frame for OCR
            averaged_img = average_frames(frame_buffer)
            roi, (x0, y0, x1, y1) = crop_roi(averaged_img)
            
            # Preprocess
            roi_proc = preprocess_for_ocr(roi)
            
            # Run OCR
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
            
            clusters = cluster_detections_by_book(new_results)
            
            call_numbers = []
            for i, cluster in enumerate(clusters):
                call_num = build_call_number_from_cluster(cluster["items"])
                x_pos = cluster["x_pos"]
                conf_avg = sum(item[2] for item in cluster["items"]) / len(cluster["items"])
                call_numbers.append((call_num, x_pos, conf_avg))
            
            cached_results = new_results
            cached_clusters = clusters
            cached_call_numbers = call_numbers
            last_ocr_t = now
            
            print(f"  Detected {len(clusters)} books:")
            for i, (call_num, x_pos, conf) in enumerate(call_numbers):
                print(f"    {i+1}. {call_num}")

        # Visualize on latest frame
        roi, (x0, y0, x1, y1) = crop_roi(img)
        disp = img.copy()
        cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 3)
        
        colors = [(255,0,0), (0,255,255), (255,0,255), (0,165,255), (255,255,0), (128,0,128)]
        
        for idx, cluster in enumerate(cached_clusters):
            color = colors[idx % len(colors)]
            for box in cluster["all_boxes"]:
                pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
                cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=3)
        
        y_offset = 50
        cv2.putText(disp, "Shelf Order (Left to Right):", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
            y_offset += 35
            display_text = f"{i+1}. {call_num}"
            cv2.putText(disp, display_text, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        frame_count += 1
        if (now - t0) >= 1.0:
            disp_fps = frame_count / (now - t0)
            t0 = now
            frame_count = 0
        
        cv2.putText(disp, f"FPS: {disp_fps:.1f} | Books: {len(cached_clusters)}", 
                   (COLOR_W - 400, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Show buffer status
        cv2.putText(disp, f"Buffer: {len(frame_buffer)}/{FRAME_BUFFER_SIZE}", 
                   (COLOR_W - 400, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow("D455 Optimized Book Reading", disp)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('s'):
            print("\n" + "="*60)
            print("SAVED SHELF ORDER")
            print("="*60)
            for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
                print(f"{i+1}. {call_num}")
            print("="*60 + "\n")
        elif key == ord('c'):
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            capture_path = f"capture_{timestamp}.jpg"
            cv2.imwrite(capture_path, disp)
            print(f"\nFrame captured: {capture_path}\n")
        elif key == ord('e') and color_sensor is not None:
            if color_sensor.supports(rs.option.exposure):
                try:
                    current = color_sensor.get_option(rs.option.exposure)
                    color_sensor.set_option(rs.option.exposure, min(current + 10, 10000))
                    print(f"Exposure increased to: {color_sensor.get_option(rs.option.exposure)}")
                except:
                    print("Could not adjust exposure")
        elif key == ord('E') and color_sensor is not None:
            if color_sensor.supports(rs.option.exposure):
                try:
                    current = color_sensor.get_option(rs.option.exposure)
                    color_sensor.set_option(rs.option.exposure, max(current - 10, 1))
                    print(f"Exposure decreased to: {color_sensor.get_option(rs.option.exposure)}")
                except:
                    print("Could not adjust exposure")
        elif key == ord('g') and color_sensor is not None:
            if color_sensor.supports(rs.option.gain):
                try:
                    current = color_sensor.get_option(rs.option.gain)
                    color_sensor.set_option(rs.option.gain, min(current + 5, 128))
                    print(f"Gain increased to: {color_sensor.get_option(rs.option.gain)}")
                except:
                    print("Could not adjust gain")
        elif key == ord('G') and color_sensor is not None:
            if color_sensor.supports(rs.option.gain):
                try:
                    current = color_sensor.get_option(rs.option.gain)
                    color_sensor.set_option(rs.option.gain, max(current - 5, 0))
                    print(f"Gain decreased to: {color_sensor.get_option(rs.option.gain)}")
                except:
                    print("Could not adjust gain")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("FINAL SHELF ORDER")
    print("="*60)
    for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
        print(f"{i+1}. {call_num}")
    print("="*60)