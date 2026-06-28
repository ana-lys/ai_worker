#!/usr/bin/env python3
import cv2
import os
import sys
import glob

def main():
    home_dir = os.path.expanduser("~")
    base_dir = os.path.join(home_dir, "record", "glitches")
    
    if not os.path.exists(base_dir):
        print(f"No glitches found at {base_dir}")
        sys.exit(1)
        
    sessions = sorted(os.listdir(base_dir), reverse=True)
    if not sessions:
        print(f"No session folders found in {base_dir}")
        sys.exit(1)
        
    print("Available sessions:")
    for i, s in enumerate(sessions):
        print(f"[{i}] {s}")
        
    try:
        sel = int(input("Select session to view [0]: ") or "0")
    except ValueError:
        sel = 0
        
    if sel < 0 or sel >= len(sessions):
        print("Invalid selection")
        sys.exit(1)
        
    session_dir = os.path.join(base_dir, sessions[sel])
    print(f"Loading from {session_dir}...")
    
    # Find all prev images
    prev_images = sorted(glob.glob(os.path.join(session_dir, "*_A_prev.jpg")))
    
    if not prev_images:
        print("No glitch images found in this session.")
        sys.exit(1)
        
    print(f"Found {len(prev_images)} glitches.")
    print("Controls:")
    print(" [n] or [Right Arrow] : Next glitch")
    print(" [p] or [Left Arrow]  : Previous glitch")
    print(" [q] or [Esc]         : Quit")
    
    idx = 0
    while True:
        prev_path = prev_images[idx]
        base_name = prev_path.replace("_A_prev.jpg", "")
        post_path = base_name + "_B_post.jpg"
        diff_path = base_name + "_D_diff.jpg"
        
        img_prev = cv2.imread(prev_path)
        img_post = cv2.imread(post_path)
        img_diff = cv2.imread(diff_path)
        
        if img_prev is None or img_post is None or img_diff is None:
            print(f"Error loading images for {base_name}")
            idx = (idx + 1) % len(prev_images)
            continue
            
        # Put labels
        cv2.putText(img_prev, f"Glitch {idx}/{len(prev_images)-1}: A (PREV)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        cv2.putText(img_post, f"Glitch {idx}/{len(prev_images)-1}: B (POST)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
        cv2.putText(img_diff, f"Glitch {idx}/{len(prev_images)-1}: D (DIFF MAP)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)
        
        # Scale down if too big to fit on screen
        h, w = img_prev.shape[:2]
        if w > 1280:
            scale = 1280 / w
            img_prev = cv2.resize(img_prev, (0,0), fx=scale, fy=scale)
            img_post = cv2.resize(img_post, (0,0), fx=scale, fy=scale)
            img_diff = cv2.resize(img_diff, (0,0), fx=scale, fy=scale)
            
        # Stack vertically or horizontally? 
        # Since they are likely 1080p scaled down, let's stack A and B horizontally, and D below them
        top_row = cv2.hconcat([img_prev, img_post])
        
        # D needs to be padded or resized to match top_row width
        diff_resized = cv2.resize(img_diff, (top_row.shape[1], int(img_diff.shape[0] * top_row.shape[1] / img_diff.shape[1])))
        
        final_view = cv2.vconcat([top_row, diff_resized])
        
        # Optional: scale down final view if it exceeds screen height
        fh, fw = final_view.shape[:2]
        if fh > 900:
            scale = 900 / fh
            final_view = cv2.resize(final_view, (0,0), fx=scale, fy=scale)

        cv2.imshow("Glitch Viewer", final_view)
        
        key = cv2.waitKey(0)
        
        if key == 27 or key == ord('q'):
            break
        elif key == 83 or key == ord('n'): # Right arrow or n
            idx = (idx + 1) % len(prev_images)
        elif key == 81 or key == ord('p'): # Left arrow or p
            idx = (idx - 1) % len(prev_images)
            if idx < 0:
                idx = len(prev_images) - 1

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
