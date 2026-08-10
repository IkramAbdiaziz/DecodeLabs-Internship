import os
import cv2
import numpy as np
import time
import argparse

def inspect_image(img_input, threshold_max=43.0):
    """
    Core CV Quality Inspection Pipeline.
    Evaluates gear components for structural defects.
    
    Args:
        img_input (str or numpy.ndarray): Path to the image or loaded image array.
        threshold_max (float): Pixel distance threshold for classifying genuine defects.
        
    Returns:
        dict: Inspection results including verdict, defect count, annotated image, and PLC trigger.
    """
    # 1. Load image if path is given
    if isinstance(img_input, str):
        img = cv2.imread(img_input)
        if img is None:
            raise ValueError(f"Failed to load image: {img_input}")
    else:
        img = img_input.copy()
        
    # Preserve original image dimensions
    height, width = img.shape[:2]
    
    # 2. Pre-processing Pipeline
    # Step 1: Convert raw RGB to Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Step 2: Apply Gaussian Blur to eliminate high-frequency noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Step 3: Thresholding to isolate binary silhouettes
    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
    
    # 3. Topological Feature Extraction
    # Extract outermost contours only (ignoring inner bore hole)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        # No object detected
        annotated_img = img.copy()
        cv2.putText(annotated_img, "VERDICT: FAIL - NO GEAR FOUND", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return {
            "verdict": "FAIL: NO GEAR DETECTED",
            "defect_count": 0,
            "annotated_image": annotated_img,
            "plc_trigger": 1
        }
        
    # Get largest contour (the gear outline)
    gear_contour = max(contours, key=cv2.contourArea)
    
    # Calculate Convex Hull
    # returnPoints=False yields indices of contour points, required for convexityDefects
    hull_indices = cv2.convexHull(gear_contour, returnPoints=False)
    
    # Sort hull indices to ensure they are sequential (prevents OpenCV crashes)
    hull_indices = np.sort(hull_indices, axis=0)
    
    # Calculate Convexity Defects
    defects = cv2.convexityDefects(gear_contour, hull_indices)
    
    # 4. Defect Calculation & Verdict
    defect_count = 0
    annotated_img = img.copy()
    
    # Draw the contour (blue outline) and hull (thin green outline) for visual verification
    cv2.drawContours(annotated_img, [gear_contour], -1, (255, 100, 50), 2)
    
    # Convert hull indices back to points to draw the hull
    hull_pts = cv2.convexHull(gear_contour)
    cv2.polylines(annotated_img, [hull_pts], True, (0, 200, 0), 1)
    
    if defects is not None:
        for i in range(defects.shape[0]):
            # s: start index, e: end index, f: farthest point index, d: fixed-point depth
            s, e, f, d = defects[i]
            
            # Convert raw distance to real pixel distance
            actual_distance = d / 256.0
            
            # Compare to threshold parameter to identify structural anomalies
            if actual_distance > threshold_max:
                defect_count += 1
                
                # Get the farthest point coordinate (the valley of the defect)
                farthest_point = tuple(gear_contour[f][0])
                
                # Dynamically derive bounding box coordinates around the anomaly (40x40 box)
                top_left = (max(0, farthest_point[0] - 20), max(0, farthest_point[1] - 20))
                bottom_right = (min(width - 1, farthest_point[0] + 20), min(height - 1, farthest_point[1] + 20))
                
                # Draw red defect bounding box
                cv2.rectangle(annotated_img, top_left, bottom_right, (0, 0, 255), 2)
                
                # Draw text description
                label = f"ANOMALY: {actual_distance:.1f}px"
                cv2.putText(annotated_img, label, (top_left[0] - 15, top_left[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
                
                # Draw a dot at the exact deepest point of defect
                cv2.circle(annotated_img, farthest_point, 3, (0, 0, 255), -1)

    # 5. Output Verdict and PLC Signals
    if defect_count > 0:
        verdict = "FAIL: STRUCTURAL DEFECT DETECTED"
        plc_trigger = 1
        verdict_color = (0, 0, 255) # Red
        text_verdict = f"FAIL - {defect_count} DEFECT(S)"
    else:
        verdict = "PASS"
        plc_trigger = 0
        verdict_color = (0, 255, 0) # Green
        text_verdict = "PASS"
        
    # Draw verdict overlay text on output
    cv2.putText(annotated_img, f"VERDICT: {text_verdict}", (25, 45), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, verdict_color, 2, cv2.LINE_AA)
    
    # Print PLC signal trigger details
    print(f"Inspection complete. Verdict: {verdict} | Defects: {defect_count} | PLC_FAIL_TRIGGER = {plc_trigger}")
    
    return {
        "verdict": verdict,
        "defect_count": defect_count,
        "annotated_image": annotated_img,
        "plc_trigger": plc_trigger
    }

def run_webcam(threshold_max):
    print("\nStarting live webcam stream quality inspection. Press 'q' to quit.")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not access webcam feed.")
        return
        
    cv2.namedWindow("Industrial Quality Inspection - Live Feed", cv2.WINDOW_NORMAL)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab webcam frame.")
            break
            
        t_start = time.perf_counter()
        result = inspect_image(frame, threshold_max)
        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        
        # Display performance text
        annotated_frame = result["annotated_image"]
        perf_text = f"Latency: {t_elapsed:.1f}ms | PLC Trigger: {result['plc_trigger']}"
        cv2.putText(annotated_frame, perf_text, (25, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        cv2.imshow("Industrial Quality Inspection - Live Feed", annotated_frame)
        
        # Quit key setup
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    print("Webcam stream stopped.")

def main():
    parser = argparse.ArgumentParser(description="Industrial Gear Quality Inspection Computer Vision Pipeline")
    parser.add_argument("--image", type=str, help="Path to a single image file to inspect")
    parser.add_argument("--dir", type=str, help="Path to a directory of images to inspect")
    parser.add_argument("--webcam", action="store_true", help="Launch live webcam inspection stream")
    parser.add_argument("--threshold", type=float, default=43.0, help="Convexity defect distance threshold (default: 43.0)")
    
    args = parser.parse_args()
    
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output_dir = os.path.join(base_dir, "outputs")
    os.makedirs(output_dir, exist_ok=True)
    
    if args.webcam:
        run_webcam(args.threshold)
        
    elif args.image:
        print(f"Inspecting single image: {args.image}")
        result = inspect_image(args.image, args.threshold)
        
        # Save output
        filename = os.path.basename(args.image)
        output_path = os.path.join(output_dir, f"annotated_{filename}")
        cv2.imwrite(output_path, result["annotated_image"])
        print(f"Saved annotated result to: {output_path}")
        
    elif args.dir:
        print(f"Inspecting directory: {args.dir}")
        if not os.path.exists(args.dir):
            print(f"Error: Directory does not exist: {args.dir}")
            return
            
        extensions = (".png", ".jpg", ".jpeg", ".bmp")
        files = [f for f in os.listdir(args.dir) if f.lower().endswith(extensions)]
        print(f"Found {len(files)} images to process.")
        
        for file in sorted(files):
            img_path = os.path.join(args.dir, file)
            print(f"\nProcessing {file}...")
            result = inspect_image(img_path, args.threshold)
            output_path = os.path.join(output_dir, f"annotated_{file}")
            cv2.imwrite(output_path, result["annotated_image"])
            
        print(f"\nBatch folder processing completed. Outputs saved to: {output_dir}")
        
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
