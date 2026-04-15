import json
import os
import argparse

def validate_manifest(manifest_path):
    print(f"\n=== OpenLabel Manifest Validation: {os.path.basename(manifest_path)} ===")
    
    if not os.path.exists(manifest_path):
        print(f"Error: Manifest not found at {manifest_path}")
        return False
        
    valid_count = 0
    error_count = 0
    
    with open(manifest_path, 'r') as f:
        for i, line in enumerate(f):
            try:
                data = json.loads(line)
                # Check top-level key
                if 'openlabel' not in data:
                    print(f"  [Line {i}] Missing 'openlabel' root key.")
                    error_count += 1
                    continue
                
                ol = data['openlabel']
                # Check metadata
                if 'metadata' not in ol:
                    print(f"  [Line {i}] Missing 'metadata' section.")
                    error_count += 1
                    continue
                
                # Check frames
                if 'frames' not in ol:
                    print(f"  [Line {i}] Missing 'frames' section.")
                    error_count += 1
                    continue
                
                # Verify external resources exist
                for frame_id, frame_data in ol['frames'].items():
                    resources = frame_data.get('frame_properties', {}).get('external_resources', [])
                    base_dir = os.path.dirname(manifest_path)
                    for res in resources:
                        res_path = os.path.join(base_dir, res['url'])
                        if not os.path.exists(res_path):
                            print(f"  [Line {i}] Resource missing: {res['url']}")
                            error_count += 1
                
                valid_count += 1
            except Exception as e:
                print(f"  [Line {i}] JSON Parse Error: {e}")
                error_count += 1
                
    print(f"\nValidation Summary:")
    print(f"  Total Lines: {valid_count + error_count}")
    print(f"  Valid Frames: {valid_count}")
    print(f"  Errors: {error_count}")
    
    if error_count == 0 and valid_count > 0:
        print("\nPASS: OpenLabel manifest follows schema and all resources are present.")
        return True
    else:
        print("\nFAIL: Manifest validation failed.")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', required=True, help='Path to master_manifest.jsonl')
    args = parser.parse_args()
    validate_manifest(args.path)
