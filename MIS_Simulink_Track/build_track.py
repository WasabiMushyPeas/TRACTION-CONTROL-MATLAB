"""Rebuild MIS reference path from the supplied GPS-only CSV. Python + NumPy/SciPy/Matplotlib."""
from pathlib import Path
import argparse, csv, hashlib, json, shutil, zipfile
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import CubicSpline
from scipy.integrate import cumulative_trapezoid
from scipy.io import savemat, loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def build(source, out):
    out.mkdir(parents=True, exist_ok=True)
    a = np.genfromtxt(source, delimiter=',', names=True)
    assert len(a)>100 and np.all(np.diff(a['time_s'])>0)
    lat,lon=np.deg2rad(a['latitude_deg']),np.deg2rad(a['longitude_deg'])
    # WGS84 ellipsoid, constant height: isolate horizontal geometry from noisy altitude.
    earth_a=6378137.; e2=6.6943799901413165e-3
    N=earth_a/np.sqrt(1-e2*np.sin(lat)**2)
    ecef=np.c_[N*np.cos(lat)*np.cos(lon), N*np.cos(lat)*np.sin(lon), N*(1-e2)*np.sin(lat)]
    d=ecef-ecef[0]; f,l=lat[0],lon[0]
    east=np.array([-np.sin(l),np.cos(l),0])
    north=np.array([-np.sin(f)*np.cos(l),-np.sin(f)*np.sin(l),np.cos(f)])
    xy=np.c_[d@east,d@north]
    raw_s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    # Start gate perpendicular to the observed first 0.5 seconds of travel.
    j=np.searchsorted(a['time_s'],0.5); tangent=xy[j]-xy[0];tangent/=np.linalg.norm(tangent)
    gate=xy@tangent
    candidates=np.flatnonzero((gate[:-1]<0)&(gate[1:]>=0)&(raw_s[:-1]>.8*raw_s[-1]))
    assert len(candidates)==1, 'Closure needs manual review: expected one final start-gate crossing.'
    i=candidates[0];alpha=-gate[i]/(gate[i+1]-gate[i])
    cross=xy[i]+alpha*(xy[i+1]-xy[i]); cut_s=raw_s[i]+alpha*(raw_s[i+1]-raw_s[i])
    cut_t=a['time_s'][i]+alpha*(a['time_s'][i+1]-a['time_s'][i])
    assert np.linalg.norm(cross)<3, 'Closure mismatch exceeds 3 m.'
    u=np.linspace(0,cut_s,int(np.ceil(cut_s/.5))+1)
    path=np.c_[np.interp(u,raw_s,xy[:,0]),np.interp(u,raw_s,xy[:,1])]
    # Meet at midpoint of gate endpoints, taper corrections over 10 m on each side.
    mid=cross/2
    for endpoint,delta,dist in [(0,mid,u),(-1,mid-cross,cut_s-u)]:
        weight=np.where(dist<10,0.5*(1+np.cos(np.pi*np.minimum(dist,10)/10)),0)
        path+=weight[:,None]*delta
    path[-1]=path[0]
    # 2 m Gaussian sigma is a modeling choice, not a GPS uncertainty estimate.
    smooth=gaussian_filter1d(path[:-1],sigma=2/(u[1]-u[0]),axis=0,mode='wrap')
    smooth=np.vstack([smooth,smooth[0]])
    cs=CubicSpline(u,smooth,axis=0,bc_type='periodic')
    dense_u=np.linspace(0,cut_s,int(np.ceil(cut_s/.02))+1)
    dense_s=cumulative_trapezoid(np.linalg.norm(cs(dense_u,1),axis=1),dense_u,initial=0)
    L=float(dense_s[-1]); s=np.linspace(0,L,int(np.ceil(L/.5))+1); q=np.interp(s,dense_s,dense_u)
    p=cs(q);v=cs(q,1);acc=cs(q,2)
    psi=np.unwrap(np.arctan2(v[:,1],v[:,0]));k=(v[:,0]*acc[:,1]-v[:,1]*acc[:,0])/np.linalg.norm(v,axis=1)**3
    t=np.interp(q,raw_s,a['time_s'])
    fields={'s_m':s,'x_m':p[:,0],'y_m':p[:,1],'z_m':np.zeros_like(s),
            'heading_rad':psi,'heading_cos':np.cos(psi),'heading_sin':np.sin(psi),
            'curvature_1pm':k,'speed_recorded_mps':np.interp(q,raw_s,a['speed_kmh'])/3.6,
            'gps_altitude_recorded_m':np.interp(q,raw_s,a['altitude_m']),
            'time_recorded_s':t,'gps_hdop':np.interp(q,raw_s,a['hdop'])}
    # Exact periodic seam for geometry. Recorded telemetry intentionally remains nonperiodic.
    for name in ['x_m','y_m','z_m','heading_cos','heading_sin','curvature_1pm']:fields[name][-1]=fields[name][0]
    deviation=np.linalg.norm(cs(u)-np.c_[np.interp(u,raw_s,xy[:,0]),np.interp(u,raw_s,xy[:,1])],axis=1)
    turn=float((psi[-1]-psi[0])/(2*np.pi))
    stats={'source_rows':len(a),'output_rows':len(s),'raw_path_length_m':float(raw_s[-1]),
           'length_m':L,'spacing_m':float(s[1]-s[0]),'raw_endpoint_gap_m':float(np.linalg.norm(xy[-1])),
           'gate_closure_gap_m':float(np.linalg.norm(cross)),'lap_cut_time_s':float(cut_t),
           'trimmed_time_s':float(a['time_s'][-1]-cut_t),'rms_fit_displacement_m':float(np.sqrt(np.mean(deviation**2))),
           'max_fit_displacement_m':float(deviation.max()),'max_abs_curvature_1pm':float(np.max(np.abs(k))),
           'net_heading_turns':turn,'integrated_curvature_rad':float(np.trapezoid(k,s)),
           'matlab_execution':'Not executed: MATLAB/Simulink unavailable in build environment.'}
    meta={'name':'MIS GPS reference 2026-06-20','schema_version':'1.0','closed':True,'length_m':L,
          'frame':'local ENU: x east, y north, z up; fixed WGS84 tangent origin',
          'origin_latitude_deg':float(a['latitude_deg'][0]),'origin_longitude_deg':float(a['longitude_deg'][0]),
          'origin_ellipsoid_height_m':0.,'height_note':'Constant ellipsoid height 0 used for horizontal projection only; GPS altitude datum unknown.',
          'heading_convention':'radians counterclockwise from east; unwrapped; positive curvature is left turn',
          'geometry_type':'smoothed driven path, not surveyed centerline',
          'elevation_mode':'flat assumption; recorded GPS altitude preserved but not used for road grade',
          'track_width_known':False,'banking_known':False,'gaussian_sigma_m':2.,'closure_blend_length_m':10.,
          'source_file':source.name,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
          'quality_note':'HDOP is dimensionless geometry dilution, not position error in meters; absolute accuracy unverified.',
          'validation':stats}
    track={**fields,'length_m':L,'closed':True,'meta':meta}
    savemat(out/'mis_track.mat',{'track':track},format='5',do_compression=True,oned_as='column',long_field_names=True)
    with (out/'mis_track.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(fields);w.writerows(zip(*fields.values()))
    (out/'mis_track.json').write_text(json.dumps({'metadata':meta,'columns':{n:v.tolist() for n,v in fields.items()}},indent=2))
    (out/'validation.json').write_text(json.dumps(stats,indent=2))
    shutil.copy2(source,out/'source_gps.csv')
    # Meaningful numerical checks and MAT round-trip validation.
    m=loadmat(out/'mis_track.mat',simplify_cells=True)['track']
    for name,values in fields.items():
        assert np.all(np.isfinite(values)) and np.array_equal(m[name],values)
    assert np.all(np.diff(s)>0) and abs(turn-round(turn))<1e-8
    assert abs(np.trapezoid(k,s)-(psi[-1]-psi[0]))<.03
    assert np.max(np.linalg.norm(np.diff(p,axis=0),axis=1))<=.501
    assert deviation.max()<2 and np.max(np.abs(k))<.5
    assert np.allclose(cs(0,1),cs(cut_s,1)) and np.allclose(cs(0,2),cs(cut_s,2))
    plt.rcParams.update({'font.family':'DejaVu Sans','axes.spines.top':False,'axes.spines.right':False,'font.size':10})
    fig=plt.figure(figsize=(12,8),facecolor='#f6f8fb');gs=fig.add_gridspec(2,2,width_ratios=[1,1.2],hspace=.38,wspace=.3)
    ax=fig.add_subplot(gs[:,0]);ax.plot(xy[:,0],xy[:,1],color='#9da9b8',lw=3,label='Recorded GPS');ax.plot(p[:,0],p[:,1],color='#1769aa',lw=1.4,label='Smooth reference')
    ax.scatter(p[0,0],p[0,1],color='#d05a24',s=45,zorder=4,label='Start / finish')
    for j in np.linspace(100,len(s)-100,5,dtype=int):ax.annotate('',xy=p[j+12],xytext=p[j],arrowprops={'arrowstyle':'->','color':'#1769aa','lw':2})
    ax.set_aspect('equal');ax.set_xlabel('East (m)');ax.set_ylabel('North (m)');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
    ax2=fig.add_subplot(gs[0,1]);ax2.plot(s,k,color='#1769aa',lw=1.2);ax2.axhline(0,color='#718096',lw=.7);ax2.set_xlabel('Distance along path (m)');ax2.set_ylabel('Signed curvature (1/m)');ax2.set_title('Positive = left turn');ax2.grid(alpha=.2)
    ax3=fig.add_subplot(gs[1,1]);ax3.plot(s,fields['speed_recorded_mps']*3.6,color='#249780');ax3.set_xlabel('Distance along path (m)');ax3.set_ylabel('Recorded speed (km/h)');ax3.set_title('Observed speed, not a target speed profile');ax3.grid(alpha=.2)
    fig.suptitle('MIS | GPS-derived track reference',fontsize=20,x=.09,ha='left',weight='bold')
    fig.text(.09,.91,f'{L:.1f} m lap  |  {len(s):,} samples  |  {s[1]-s[0]:.3f} m spacing  |  Closed smooth path',color='#4c5c70')
    fig.text(.09,.035,'Driven path only. Width, boundaries, banking and road elevation are not established by this GPS trace.',fontsize=9,color='#4c5c70')
    fig.subplots_adjust(top=.86,bottom=.12);fig.savefig(out/'track_preview.png',dpi=160);plt.close(fig)
    print(json.dumps(stats,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=Path(__file__).parent/'source_gps.csv');parser.add_argument('--out',type=Path,default=Path(__file__).parent/'rebuilt');args=parser.parse_args();build(args.source,args.out)
