import numpy as np
import open3d as o3d
import cv2
import sys
import os
import matplotlib.pyplot as plt

def main():
    if len(sys.argv) < 2:
        print("Usage: extract_lines.py <path_to_pcd>")
        sys.exit(1)
        
    pcd_path = sys.argv[1]
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)[:, :2]
    
    # 1. Convert to 2D image (Occupancy Grid)
    resolution = 0.02 # 2cm per pixel
    min_x, min_y = np.min(pts, axis=0)
    max_x, max_y = np.max(pts, axis=0)
    
    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    
    grid = np.zeros((height, width), dtype=np.uint8)
    
    # Project points to grid
    px = ((pts[:, 0] - min_x) / resolution).astype(int)
    py = ((pts[:, 1] - min_y) / resolution).astype(int)
    
    grid[py, px] = 255
    
    # Optional: Dilate slightly to connect sparse points on the same line
    kernel = np.ones((3,3), np.uint8)
    grid_dilated = cv2.dilate(grid, kernel, iterations=1)
    
    # 2. Extract lines using Probabilistic Hough Transform (STRICTER)
    lines = cv2.HoughLinesP(
        grid_dilated, 
        rho=1, 
        theta=np.pi/180, 
        threshold=50,      # Increased from 30: Need more collinear points to form a line
        minLineLength=50,  # Increased from 30: Minimum 1 meter (50 * 2cm) long to avoid small islands
        maxLineGap=5       # Decreased from 10: Maximum gap of 10cm (5 * 2cm) to still be considered one line
    )
    
    if lines is None:
        print("No lines found!")
        sys.exit(1)
        
    print(f"Extracted {len(lines)} line segments.")
    
    # 3. Plot the result
    plt.figure(figsize=(12, 12), dpi=150)
    # Plot original points in light grey
    plt.scatter(pts[:, 0], pts[:, 1], c='lightgray', s=1, label='Point Cloud')
    
    colors = plt.cm.jet(np.linspace(0, 1, len(lines)))
    
    for i, line in enumerate(lines):
        x1, y1, x2, y2 = line[0]
        # Convert pixel coordinates back to real world meters
        rx1 = x1 * resolution + min_x
        ry1 = y1 * resolution + min_y
        rx2 = x2 * resolution + min_x
        ry2 = y2 * resolution + min_y
        
        plt.plot([rx1, rx2], [ry1, ry2], color=colors[i], linewidth=2)
        
    plt.axis('equal')
    plt.grid(True)
    plt.title(f"Extracted Walls (Hough Transform: {len(lines)} segments)")
    plt.legend()
    
    out_path = '/home/lys/.gemini/antigravity-ide/brain/a973bde8-83a1-4ecc-baa5-e9a52abfb9e7/extracted_lines.png'
    plt.savefig(out_path, bbox_inches='tight')
    print(f"Saved lines plot to {out_path}")

if __name__ == '__main__':
    main()
