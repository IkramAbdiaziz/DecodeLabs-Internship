import os
import cv2
import time
import numpy as np
from inspection import inspect_image

def run_batch_evaluation():
    # 1. Setup paths
    src_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(src_dir)
    pass_dir = os.path.join(project_dir, "dataset", "pass")
    fail_dir = os.path.join(project_dir, "dataset", "fail")
    output_dir = os.path.join(project_dir, "outputs")
    
    os.makedirs(output_dir, exist_ok=True)
    
    # 2. Gather image lists
    pass_images = sorted([f for f in os.listdir(pass_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
    fail_images = sorted([f for f in os.listdir(fail_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
    
    total_images = len(pass_images) + len(fail_images)
    print("=" * 80)
    print(f"STARTING BATCH QUALITY INSPECTION EVALUATION ({total_images} total images)")
    print(f"PASS Control Directory: {pass_dir}")
    print(f"FAIL Defect Directory:  {fail_dir}")
    print("=" * 80)
    
    # Track statistics
    correct_classifications = 0
    total_latency_ms = 0.0
    total_defects_detected = 0
    
    # To store detailed run data for tabular output
    evaluation_table = []
    
    # 3. Process PASS Images (Expected Verdict: PASS)
    print("\nProcessing PASS Control Set...")
    for filename in pass_images:
        path = os.path.join(pass_dir, filename)
        
        t_start = time.perf_counter()
        result = inspect_image(path, threshold_max=43.0)
        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        
        # Save output image
        cv2.imwrite(os.path.join(output_dir, f"annotated_{filename}"), result["annotated_image"])
        
        actual_verdict = result["verdict"]
        defect_count = result["defect_count"]
        is_correct = (actual_verdict == "PASS")
        
        if is_correct:
            correct_classifications += 1
            
        total_latency_ms += t_elapsed_ms
        total_defects_detected += defect_count
        
        evaluation_table.append({
            "filename": filename,
            "category": "PASS (Control)",
            "expected": "PASS",
            "actual": "PASS" if actual_verdict == "PASS" else "FAIL",
            "defects": defect_count,
            "latency": t_elapsed_ms,
            "status": "SUCCESS" if is_correct else "MISCLASSIFIED"
        })
        
    # 4. Process FAIL Images (Expected Verdict: FAIL: STRUCTURAL DEFECT DETECTED)
    print("\nProcessing FAIL Defective Set...")
    for filename in fail_images:
        path = os.path.join(fail_dir, filename)
        
        t_start = time.perf_counter()
        result = inspect_image(path, threshold_max=43.0)
        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        
        # Save output image
        cv2.imwrite(os.path.join(output_dir, f"annotated_{filename}"), result["annotated_image"])
        
        actual_verdict = result["verdict"]
        defect_count = result["defect_count"]
        is_correct = (actual_verdict == "FAIL: STRUCTURAL DEFECT DETECTED")
        
        if is_correct:
            correct_classifications += 1
            
        total_latency_ms += t_elapsed_ms
        total_defects_detected += defect_count
        
        evaluation_table.append({
            "filename": filename,
            "category": "FAIL (Defective)",
            "expected": "FAIL",
            "actual": "FAIL" if actual_verdict == "FAIL: STRUCTURAL DEFECT DETECTED" else "PASS",
            "defects": defect_count,
            "latency": t_elapsed_ms,
            "status": "SUCCESS" if is_correct else "MISCLASSIFIED"
        })

    # 5. Display tabular report
    print("\n" + "=" * 90)
    print(f"{'IMAGE NAME':<20} | {'CATEGORY':<16} | {'EXPECTED':<8} | {'ACTUAL':<8} | {'DEFECTS':<7} | {'LATENCY (ms)':<12} | {'VERDICT STATUS'}")
    print("-" * 90)
    for entry in evaluation_table:
        print(f"{entry['filename']:<20} | {entry['category']:<16} | {entry['expected']:<8} | {entry['actual']:<8} | {entry['defects']:<7} | {entry['latency']:<12.2f} | {entry['status']}")
    print("=" * 90)
    
    # 6. Print Summary Metrics
    accuracy_pct = (correct_classifications / total_images) * 100.0
    avg_latency_ms = total_latency_ms / total_images
    
    print("\n" + "=" * 45)
    print("           QUALITY EVALUATION SUMMARY        ")
    print("-" * 45)
    print(f" Total Images Evaluated:       {total_images}")
    print(f" Correctly Classified:         {correct_classifications}/{total_images}")
    print(f" Batch Classification Accuracy: {accuracy_pct:.1f}%")
    print(f" Average Latency per Frame:    {avg_latency_ms:.2f} ms")
    print(f" Total Anomalies Detected:     {total_defects_detected}")
    print(f" Outputs saved in:             {output_dir}")
    print("=" * 45)

if __name__ == "__main__":
    run_batch_evaluation()
