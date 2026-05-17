"""Debug script: trace the root orientation pipeline for a standing person."""
import numpy as np
from scipy.spatial.transform import Rotation as sRot
import torch
import sys
sys.path.insert(0, "/home/ustczxh/humanoid/GR00T-WholeBodyControl")

from gear_sonic.trl.utils.torch_transform import angle_axis_to_quaternion
from gear_sonic.isaac_utils.rotations import smpl_root_ytoz_up, remove_smpl_base_rot

def wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]])

def trace_root(label, root_aa_np):
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    
    global_orient = torch.from_numpy(root_aa_np).float().unsqueeze(0)
    print(f"  root axis-angle:      {global_orient[0].numpy()}")
    
    global_orient_quat = angle_axis_to_quaternion(global_orient)
    print(f"  root quat (wxyz):     {global_orient_quat[0].numpy()}")
    
    after_ytoz = smpl_root_ytoz_up(global_orient_quat)
    print(f"  after ytoz_up (wxyz): {after_ytoz[0].numpy()}")
    
    after_base = remove_smpl_base_rot(after_ytoz, w_last=False)
    print(f"  after base_rot(wxyz): {after_base[0].numpy()}")
    
    q_wxyz = after_base[0].numpy()
    q_xyzw = wxyz_to_xyzw(q_wxyz)
    r = sRot.from_quat(q_xyzw)
    euler = r.as_euler("xyz", degrees=True)
    print(f"  final euler (xyz deg): roll={euler[0]:.1f}, pitch={euler[1]:.1f}, yaw={euler[2]:.1f}")
    
    forward = r.apply([1, 0, 0])
    up = r.apply([0, 0, 1])
    print(f"  +X maps to: [{forward[0]:.3f}, {forward[1]:.3f}, {forward[2]:.3f}]")
    print(f"  +Z maps to: [{up[0]:.3f}, {up[1]:.3f}, {up[2]:.3f}]")
    print(f"  (Z-up robot frame: +X=forward, +Z=up)")
    
    if abs(forward[0]) > 0.9 and abs(up[2]) > 0.9:
        sign = "forward" if forward[0] > 0 else "backward"
        print(f"  ✅ Standing upright, facing {sign}")
    else:
        print(f"  ❌ NOT standing upright facing forward")

print("Testing root orientation corrections for standing person in Unity Y-up")

trace_root("No correction (identity)", np.array([0, 0, 0], dtype=np.float32))

r180y = sRot.from_euler("y", 180, degrees=True)
trace_root("180° Y on root", r180y.as_rotvec().astype(np.float32))

r90y = sRot.from_euler("y", 90, degrees=True)
trace_root("90° Y on root", r90y.as_rotvec().astype(np.float32))

r_neg90y = sRot.from_euler("y", -90, degrees=True)
trace_root("-90° Y on root", r_neg90y.as_rotvec().astype(np.float32))

# Also test: what if we DON'T apply remove_smpl_base_rot?
print(f"\n{'='*60}")
print(f"  Without remove_smpl_base_rot (identity root)")
print(f"{'='*60}")
global_orient = torch.zeros(1, 3)
q = angle_axis_to_quaternion(global_orient)
q_z = smpl_root_ytoz_up(q)
q_wxyz = q_z[0].numpy()
q_xyzw = wxyz_to_xyzw(q_wxyz)
r = sRot.from_quat(q_xyzw)
forward = r.apply([1, 0, 0])
up = r.apply([0, 0, 1])
print(f"  quat (wxyz): {q_wxyz}")
print(f"  +X maps to: [{forward[0]:.3f}, {forward[1]:.3f}, {forward[2]:.3f}]")
print(f"  +Z maps to: [{up[0]:.3f}, {up[1]:.3f}, {up[2]:.3f}]")

# SMPL base rot info
base = sRot.from_quat([0.5, 0.5, 0.5, 0.5])  # xyzw
print(f"\nSMPL base_rot [0.5,0.5,0.5,0.5] xyzw:")
print(f"  euler xyz: {base.as_euler('xyz', degrees=True)}")
