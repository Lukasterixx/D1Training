"""Both real-frame replays for each exported armft6 checkpoint: the gate."""
import glob, os, re, subprocess, sys
A='logs/unifp_train/armlink'
L=sorted(glob.glob(f'{A}/reach_6[bc]_*.npz'),key=lambda p:(p.split('_')[-2],int(re.search(r'_(\d+)\.npz',p).group(1))))
a=subprocess.run([sys.executable,f'{A}/semiclosed.py',f'{A}/policy_rows.npz',*L],capture_output=True,text=True).stdout.splitlines()[1:]
b=subprocess.run([sys.executable,f'{A}/stance2.py',f'{A}/policy_rows_dry.npz',*L],capture_output=True,text=True).stdout.splitlines()
c=subprocess.run([sys.executable,f'{A}/semiclosed.py',f'{A}/policy_rows_wood.npz',*L],capture_output=True,text=True).stdout.splitlines()[1:]
print(f"{'':16s} carpet: lead 1 s+  held | wood: lead 1 s+  held | stand: lead 1 s+  J4 at 20 s | gate")
for x,y,w in zip(a,b,c):
    n=x.split()[0].replace('reach_','')
    j4=[int(v) for v in re.search(r'J4 (.*?) \|',x).group(1).split()]
    l1=int(re.search(r'lead 1 s\+\s+(\d+)',x).group(1)); l2=int(re.search(r'lead (\d+)$',x.strip()).group(1)); l3=int(re.search(r'lead past 1 s\s+(\d+)',y).group(1)); j20=int(re.search(r'20 s\s+(-?\d+) \|',y).group(1))
    w1=int(re.search(r'lead 1 s\+\s+(\d+)',w).group(1)); w2=int(re.search(r'lead (\d+)$',w.strip()).group(1))
    print(f"{n:16s} {l1:12d} {l2:5d} | {w1:10d} {w2:5d} | {l3:11d} {j20:8d}    | {'PASS' if max(l1,l2,l3,w1,w2)<=15 else ''}")
