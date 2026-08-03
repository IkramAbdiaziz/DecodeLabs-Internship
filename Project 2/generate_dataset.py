import os
import cv2
import numpy as np

def create_gear_image(is_defective=False, defect_type="missing_teeth", rotation_deg=0.0, scale=1.0, offset=(0, 0)):
    # Create image canvas (500x500)
    width, height = 500, 500
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # 1. Apply a subtle radial gradient background to simulate industrial ring-light illumination
    y_coords, x_coords = np.mgrid[0:height, 0:width]
    dist_from_center = np.sqrt((x_coords - width//2)**2 + (y_coords - height//2)**2)
    # Brightest near center/conveyor surface, falling off towards edges
    bg_gradient = np.clip(40 - dist_from_center * 0.08, 10, 45).astype(np.uint8)
    for c in range(3):
        img[:, :, c] = bg_gradient

    # Gear Base parameters
    cx = width // 2 + offset[0]
    cy = height // 2 + offset[1]
    
    # Base Radii scaled
    R_inner = int(120 * scale)
    R_outer = int(160 * scale)
    
    # Number of teeth
    N_teeth = 18
    
    # Calculate gear contour points
    points = []
    num_steps = 720
    
    # Convert rotation to radians
    rot_rad = np.radians(rotation_deg)
    
    for i in range(num_steps):
        # Calculate angle with rotation offset
        theta = 2 * np.pi * i / num_steps
        angle_eval = (theta + rot_rad) % (2 * np.pi)
        
        # Base periodic tooth pattern (trapezoidal wave)
        tooth_phase = (N_teeth * angle_eval) % (2 * np.pi)
        t = tooth_phase / (2 * np.pi)
        
        # Trapezoidal profile: 
        # 0.0 to 0.4: top (outer tooth tip)
        # 0.4 to 0.5: slope down
        # 0.5 to 0.9: bottom (inner gap)
        # 0.9 to 1.0: slope up
        if t < 0.4:
            r = R_outer
        elif t < 0.5:
            r = R_outer - (R_outer - R_inner) * (t - 0.4) / 0.1
        elif t < 0.9:
            r = R_inner
        else:
            r = R_inner + (R_outer - R_inner) * (t - 0.9) / 0.1
            
        # Apply structural defects if fail mode is active
        if is_defective:
            # Note: evaluate defects in stationary coordinate frame so they don't depend on rotation angle
            if defect_type == "missing_teeth":
                # Omit teeth in a 60-degree sector (0 to pi/3) and make a deeper recess to simulate broken teeth root
                if 0.0 <= theta <= np.pi / 3:
                    r = R_inner - int(25 * scale)
            elif defect_type == "crack":
                # A sharp triangular fracture cutting deep into the core at 180 degrees (pi)
                crack_width = 0.08
                if np.pi - crack_width <= theta <= np.pi + crack_width:
                    depth_scale = 1.0 - abs(theta - np.pi) / crack_width
                    r = R_inner - (80 * scale) * depth_scale
                    
        # Compute coordinates
        x = int(cx + r * np.cos(theta))
        y = int(cy + r * np.sin(theta))
        points.append([x, y])
        
    pts = np.array(points, dtype=np.int32)
    
    # 2. Draw Gear Silhouette (metallic matte grey)
    cv2.fillPoly(img, [pts], (180, 185, 190))
    # Draw dark inner gear details (rim lines to simulate thickness)
    cv2.polylines(img, [pts], True, (130, 135, 140), 2)
    
    # 3. Draw Center Bore Hole and Keyway (standard manufacturing details)
    bore_r = int(35 * scale)
    # Bore hole polygon
    bore_pts = []
    num_bore_steps = 360
    for i in range(num_bore_steps):
        b_theta = 2 * np.pi * i / num_bore_steps
        br = bore_r
        
        # Add a square keyway slot at top of bore hole (stationary relative to gear core)
        # Apply keyway at 270 degrees (straight up in standard frame)
        b_eval = (b_theta + rot_rad) % (2 * np.pi)
        if 1.45 * np.pi <= b_eval <= 1.55 * np.pi:
            br = bore_r + (12 * scale)
            
        bx = int(cx + br * np.cos(b_theta))
        by = int(cy + br * np.sin(b_theta))
        bore_pts.append([bx, by])
        
    bore_poly = np.array(bore_pts, dtype=np.int32)
    # Fill bore hole with dark background shadow
    cv2.fillPoly(img, [bore_poly], (15, 15, 15))
    cv2.polylines(img, [bore_poly], True, (80, 80, 80), 2)
    
    # 4. Add sensor noise (Gaussian noise) to make it look like a real industrial camera feed
    noise = np.random.normal(0, 1.5, img.shape).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    
    return img

def main():
    # Setup directories
    base_dir = os.path.dirname(os.path.abspath(__file__))
    pass_dir = os.path.join(base_dir, "dataset", "pass")
    fail_dir = os.path.join(base_dir, "dataset", "fail")
    outputs_dir = os.path.join(base_dir, "outputs")
    
    os.makedirs(pass_dir, exist_ok=True)
    os.makedirs(fail_dir, exist_ok=True)
    os.makedirs(outputs_dir, exist_ok=True)
    
    print("Generating synthetic gear dataset...")
    
    # Generate 10 PASS Gears (uniform, rotated, scaled, translated)
    for i in range(10):
        # Varying parameters
        rot = np.random.uniform(0, 360)
        sc = np.random.uniform(0.92, 1.08)
        off_x = np.random.randint(-12, 12)
        off_y = np.random.randint(-12, 12)
        
        img = create_gear_image(
            is_defective=False,
            rotation_deg=rot,
            scale=sc,
            offset=(off_x, off_y)
        )
        
        filename = os.path.join(pass_dir, f"gear_pass_{i+1:02d}.png")
        cv2.imwrite(filename, img)
        print(f"Saved: {filename} (Rot: {rot:.1f}deg, Scale: {sc:.2f}, Off: ({off_x}, {off_y}))")

    # Generate 10 FAIL Gears (5 missing teeth, 5 cracked cores)
    for i in range(10):
        rot = np.random.uniform(0, 360)
        sc = np.random.uniform(0.92, 1.08)
        off_x = np.random.randint(-12, 12)
        off_y = np.random.randint(-12, 12)
        
        # Alternate defect type
        def_type = "missing_teeth" if i % 2 == 0 else "crack"
        
        img = create_gear_image(
            is_defective=True,
            defect_type=def_type,
            rotation_deg=rot,
            scale=sc,
            offset=(off_x, off_y)
        )
        
        filename = os.path.join(fail_dir, f"gear_fail_{i+1:02d}.png")
        cv2.imwrite(filename, img)
        print(f"Saved: {filename} (Defect: {def_type}, Rot: {rot:.1f}deg, Scale: {sc:.2f})")
        
    print("\nDataset generation complete!")
    print(f"PASS directory: {pass_dir}")
    print(f"FAIL directory: {fail_dir}")

if __name__ == "__main__":
    main()
