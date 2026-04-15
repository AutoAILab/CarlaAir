import os
import argparse
from PIL import Image

def create_sync_gif(data_dir, output_name='sync_check.gif', sample_rate=10):
    images_dir = os.path.join(data_dir, 'images')
    verification_dir = os.path.join(data_dir, 'verification')
    os.makedirs(verification_dir, exist_ok=True)
    
    # Find all frame IDs based on UGV images
    ugv_files = sorted([f for f in os.listdir(images_dir) if f.startswith('ugv_') and f.endswith('.png')])
    frame_ids = [int(f.split('_')[1].split('.')[0]) for f in ugv_files]
    
    # Sample frames
    sampled_ids = frame_ids[::sample_rate]
    print(f"Sampling {len(sampled_ids)} frames for Sync GIF (2Hz equivalent)...")
    
    frames = []
    for fid in sampled_ids:
        ugv_path = os.path.join(images_dir, f'ugv_{fid:06d}.png')
        uav_path = os.path.join(images_dir, f'UAV_1_{fid:06d}.png')
        
        if not os.path.exists(ugv_path) or not os.path.exists(uav_path):
            print(f"Warning: Skipping frame {fid}, missing image.")
            continue
            
        img_ugv = Image.open(ugv_path)
        img_uav = Image.open(uav_path)
        
        # Resize to same height if needed (assume same for now)
        # Create side-by-side
        total_width = img_ugv.width + img_uav.width
        max_height = max(img_ugv.height, img_uav.height)
        
        combined = Image.new('RGB', (total_width, max_height))
        combined.paste(img_ugv, (0, 0))
        combined.paste(img_uav, (img_ugv.width, 0))
        
        # Add frame ID text overlay (optional, but good for debugging)
        # For simplicity, we skip text overlay to avoid dependency on fonts in PIL
        
        frames.append(combined)
    
    if frames:
        output_path = os.path.join(verification_dir, output_name)
        frames[0].save(output_path, format='GIF', append_images=frames[1:], save_all=True, duration=500, loop=0)
        print(f"Sync GIF saved to: {output_path}")
    else:
        print("Error: No frames found to create GIF.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True, help='Dataset output directory')
    parser.add_argument('--out', default='sync_check.gif', help='Output GIF name')
    parser.add_argument('--rate', type=int, default=10, help='Sampling rate (default: 10 for 2Hz at 20FPS)')
    args = parser.parse_args()
    
    create_sync_gif(args.dir, args.out, args.rate)
