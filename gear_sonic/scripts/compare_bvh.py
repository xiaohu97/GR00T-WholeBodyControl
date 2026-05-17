"""Compare last frames of two BVH files to identify rotation differences."""
import sys

# soma BVH joint order (from the hierarchy):
# Root(6) Hips(6) Spine1(3) Spine2(3) Chest(3)
# Neck1(3) Neck2(3) Head(3) HeadEnd(3) Jaw(3) LeftEye(3) RightEye(3)
# LeftCollar(3) LeftShoulder(3) LeftElbow(3) LeftWrist(3) 
# + finger chains...
# RightCollar(3) RightShoulder(3) RightElbow(3) RightWrist(3)
# + finger chains...
# LeftHip(3) LeftKnee(3) LeftAnkle(3) LeftToe(3) LeftToeEnd(3)
# RightHip(3) RightKnee(3) RightAnkle(3) RightToe(3) RightToeEnd(3)
# LeftThumb/Index/Middle/Ring/Pinky chains (L hand)
# RightThumb/Index/Middle/Ring/Pinky chains (R hand)

# Key joints and their channel offsets (in order of appearance in MOTION data)
# Root: 0-5 (pos + ZYX euler)
# Hips: 6-11 (pos + ZYX euler)  
# Spine1: 12-14 (ZYX euler)
# Spine2: 15-17
# Chest: 18-20
# Neck1: 21-23
# Neck2: 24-26
# Head: 27-29

JOINTS = [
    ("Root",      0, 6),   # 6 channels: pos(3) + rot(3)
    ("Hips",      6, 6),   # 6 channels: pos(3) + rot(3) 
    ("Spine1",   12, 3),
    ("Spine2",   15, 3),
    ("Chest",    18, 3),
    ("Neck1",    21, 3),
    ("Neck2",    24, 3),
    ("Head",     27, 3),
    ("HeadEnd",  30, 3),
    ("Jaw",      33, 3),
    ("LeftEye",  36, 3),
    ("RightEye", 39, 3),
]

def parse_last_frame(filepath):
    with open(filepath) as f:
        lines = f.readlines()
    # Find last non-empty line
    for line in reversed(lines):
        line = line.strip()
        if line and not line.startswith(('HIERARCHY', 'ROOT', 'JOINT', '{', '}', 'OFFSET', 'CHANNELS', 'End', 'MOTION', 'Frames', 'Frame')):
            values = [float(x) for x in line.split()]
            return values
    return None

ref_file = "pico_records/high_jump_R_001__A277.bvh"
new_file = "pico_records/soma_session_20260517_234621.bvh"

ref = parse_last_frame(ref_file)
new = parse_last_frame(new_file)

print(f"Reference: {ref_file} ({len(ref)} values)")
print(f"New:       {new_file} ({len(new)} values)")

print(f"\n{'Joint':<12} {'Chan':>4}  {'Reference':>40}  {'New':>40}  {'Diff':>30}")
print("-" * 140)

for name, offset, nchan in JOINTS:
    ref_vals = ref[offset:offset+nchan]
    new_vals = new[offset:offset+nchan]
    diff = [n - r for r, n in zip(ref_vals, new_vals)]
    
    if nchan == 6:
        labels = "px,py,pz,rZ,rY,rX"
    else:
        labels = "rZ,rY,rX"
    
    ref_str = ", ".join(f"{v:8.2f}" for v in ref_vals)
    new_str = ", ".join(f"{v:8.2f}" for v in new_vals)
    diff_str = ", ".join(f"{v:+8.2f}" for v in diff)
    
    print(f"{name:<12} [{labels}]")
    print(f"  REF: {ref_str}")
    print(f"  NEW: {new_str}")
    print(f"  DIF: {diff_str}")
    print()

# Also show the first ~45 channels to capture upper body
print("\n=== Full first 90 values comparison ===")
print(f"{'Idx':>4} {'Reference':>12} {'New':>12} {'Diff':>12}")
for i in range(min(90, len(ref), len(new))):
    diff = new[i] - ref[i]
    marker = " <<<" if abs(diff) > 5 else ""
    print(f"{i:4d} {ref[i]:12.3f} {new[i]:12.3f} {diff:+12.3f}{marker}")
