"""Offline replay of the RSSI heading tracker (src/donut_rssi_heading.c) on a dumped log.

Rebuilds the tracker from the logged R (raw RSSI, ~1 kHz), P (RSSI rpm estimate) and C (corrected
accel rpm) records and compares variants, printing lock percentage, rejected updates and the
per-update phase error spread while locked. "accel" mode (speed = scale * corrected accel rpm,
RSSI pins the angle and slowly the scale, KP 0.5 / KS 0.005) is what the firmware does now;
"pll" mode is the earlier phase/frequency loop.

Usage: python tools/rssi_heading_replay.py heading_shadow.csv 8.2 34.8   (log, start s, end s)
Needs numpy. Python replica of the firmware logic, not the compiled code.
"""
import csv, bisect, numpy as np, collections, sys

H, STEPS, WINDOW_US, UPDATE_US = 4, 72, 200000, 50000

def load(path):
    rows=list(csv.DictReader(open(path)))
    R=[(int(r['t_us']),int(r['rssi'])) for r in rows if r['kind']=='R']
    P=[(int(r['t_us']),int(r['rssi_rpm'] or 0),float(r['confidence'] or 0)) for r in rows if r['kind']=='P']
    C=[(int(r['t_us']),int(r['accel_rpm_corrected'])) for r in rows if r['kind']=='C']
    return R,P,C

def run(data, t0, t1, mode='pll', KP=0.3, KI=0.02, KS=0.02, Q_MIN=0.5, MAX_ERR=60/360,
        max_rpm_diff=None, accel_tau=0.0, hold_us=2000000):
    # max_rpm_diff: restart (lose the lock) when the tracker rpm is this far from the RSSI rpm, like
    # the firmware's RSSI_HEADING_MAX_RPM_DIFF. accel_tau: low-pass (s) on the accel rpm used for speed.
    R,P,C=data
    rt=np.array([t for t,_ in R]); rv=np.array([v for _,v in R],float)
    ct=[t for t,_ in C]
    def accel_at(t):
        i=max(0,bisect.bisect_right(ct,t)-1); return C[i][1]
    def accel_slope(t):  # firmware-like trend: LSQ over 11 samples 50 ms apart
        h=np.array([accel_at(t-50000*k) for k in range(10,-1,-1)],float); x=np.arange(11)-5.0
        return float(np.sum(x*(h-h.mean()))/np.sum(x*x))*20
    valid=[(t,r) for t,r,c in P if r>0 and c>=0.2]; vt=[v[0] for v in valid]
    def rssi_at(t):
        i=bisect.bisect_right(vt,t)-1
        return None if i<0 or t-valid[i][0]>200000 else valid[i][1]
    # start from the RSSI rpm
    r0=rssi_at(t0) or accel_at(t0)
    f=r0/60; scale=r0/max(accel_at(t0),1); phase=0.0; tmpl=None; good=bad=0; state=1
    log=[]; now=t0
    acc_f=accel_at(t0); restarts=0; last_locked=None
    while now<t1:
        now+=UPDATE_US
        k=1.0 if accel_tau<=0 else (UPDATE_US/1e6)/(accel_tau+UPDATE_US/1e6)
        acc_f+=k*(accel_at(now)-acc_f)
        if mode=='accel':
            f=scale*acc_f/60                  # speed follows the accel, RSSI pins the angle + scale
        r=rssi_at(now)
        if max_rpm_diff is not None and r and abs(f*60-r)/r>max_rpm_diff:
            # firmware: go idle, restart from the RSSI rpm at the next update (template kept)
            # firmware: re-seed speed/scale from the RSSI rpm, keep the angle, earn the lock back
            restarts+=1; good=0; state=1
            f=r/60; scale=r/max(acc_f,1)
        m=(rt>now-WINDOW_US)&(rt<=now); n=int(m.sum())
        if n<120: log.append((now,state,0,0)); continue
        ts=rt[m]; vs=rv[m]-rv[m].mean()
        # model phase at sample times (constant f over the window, as the firmware does)
        ph=(phase+f*(ts-now)/1e6)%1.0
        z=np.array([np.sum(vs*np.exp(-2j*np.pi*h*ph)) for h in range(1,H+1)])/n
        hs=np.arange(1,H+1)
        if tmpl is None:
            tmpl=z*np.exp(-1j*hs*np.angle(z[0])); log.append((now,state,0,0))
            phase=(phase+f*UPDATE_US/1e6)%1.0; continue
        w=np.conj(tmpl)*z; d=np.arange(STEPS)/STEPS-0.5
        S=np.real(np.exp(2j*np.pi*np.outer(d,hs))@w); k=int(np.argmax(S))
        a,b,c=S[k-1],S[k],S[(k+1)%STEPS]; den=a-2*b+c
        ref=max(-0.5,min(0.5,0.5*(a-c)/den)) if den<0 else 0
        shift=(k+ref)/STEPS-0.5; err=-shift
        q=b/(np.linalg.norm(tmpl)*np.linalg.norm(z)+1e-12)
        rej=q<Q_MIN or abs(err)>MAX_ERR
        if not rej:
            phase=(phase+KP*err)%1.0
            if mode=='accel':
                scale*=1+KS*err/(UPDATE_US/1e6)/max(f,1e-6)   # integral term on the scale factor
            else:
                f+=KI*err/(UPDATE_US/1e6)
            al=UPDATE_US/2e6
            tmpl=(1-al)*tmpl+al*z*np.exp(2j*np.pi*hs*shift); tmpl=tmpl*np.exp(-1j*hs*np.angle(tmpl[0]))
            good+=1; bad=0
        else:
            bad+=1; good=0
        if mode=='pll':
            f+=accel_slope(now)/60*UPDATE_US/1e6
        if state==1 and good>=10: state=2
        elif state==2 and bad>=10: state=1
        if state==2: last_locked=now
        shown=3 if state==1 and last_locked is not None and now-last_locked<hold_us else state
        log.append((now,shown,q,err*360))
        phase=(phase+f*UPDATE_US/1e6)%1.0   # advance to the next update
    return log

def summary(log, label):
    st=[l[1] for l in log]; n=len(log); restarts=st.count(0)
    usable=st.count(2)+st.count(3)
    used=np.array([l[3] for l in log if l[1]==2 and l[2]>=0.5 and abs(l[3])<=60])
    rej=sum(1 for l in log if l[2] and (l[2]<0.5 or abs(l[3])>60))
    print(f"{label:52s} locked {100*st.count(2)/n:3.0f}%  +holding {100*usable/n:3.0f}%  rejected {100*rej/n:3.0f}%  locked err std {used.std() if len(used) else float('nan'):5.1f} deg")

if __name__=='__main__':
    data=load(sys.argv[1]); a,b=float(sys.argv[2])*1e6,float(sys.argv[3])*1e6
    summary(run(data,a,b,'pll',0.3,0.02),"old PLL (KP .3 KI .02 + accel trend)")
    summary(run(data,a,b,'pll',0.4,0.08),"PLL, higher gains (KP .4 KI .08)")
    summary(run(data,a,b,'pll',0.5,0.15),"PLL, much higher gains (KP .5 KI .15)")
    for ks in (0.005,0.02,0.05):
        summary(run(data,a,b,'accel',0.3,KS=ks),f"speed from accel, RSSI pins angle (KS {ks})")
        summary(run(data,a,b,'accel',0.5,KS=ks),f"  same with KP .5 (KS {ks})" + ("  <- current firmware" if ks == 0.005 else ""))
