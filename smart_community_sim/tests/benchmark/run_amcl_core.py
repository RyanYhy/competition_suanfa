#!/usr/bin/env python3
"""Actual AMCL core on synthetic world-raycast scans, not Gazebo performance."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from generate_navigation_maps import transform


def wrap(angle):return np.arctan2(np.sin(angle),np.cos(angle))


def dataset(path,seed):
    rng=np.random.RandomState(seed)
    grid=np.flipud(np.array(Image.open(ROOT/'maps/field_reference.pgm')))
    meta=yaml.safe_load((ROOT/'maps/field_reference.yaml').read_text())
    world=E.parse(ROOT/'worlds/smart_community.world').getroot().find('world')
    boxes=[]
    for model in world.findall('model'):
        for collision in model.findall('link/collision'):
            box=collision.find('geometry/box')
            if box is None:continue
            pose=transform(model).dot(transform(collision))
            dims=np.array(list(map(float,box.findtext('size').split())))
            if abs(.265-pose[2,3])<=dims[2]/2:boxes.append((pose,dims))
    route=yaml.safe_load((ROOT/'config/navigation/patrol_route.yaml').read_text())['waypoints']
    current=np.array([1.65,1.75,math.pi]);poses=[current.copy()]
    for target in route:
        position=np.array([target['x'],target['y']])
        distance=np.linalg.norm(position-current[:2]);n=max(1,math.ceil(distance/.03))
        start=current.copy()
        for ratio in np.linspace(0,1,n+1)[1:]:poses.append(np.r_[start[:2]*(1-ratio)+position*ratio,start[2]])
        current=poses[-1].copy();yaw_delta=float(wrap(target['yaw']-current[2]));n=max(1,math.ceil(abs(yaw_delta)/.08))
        start=current.copy()
        for ratio in np.linspace(0,1,n+1)[1:]:poses.append(np.r_[position,wrap(start[2]+yaw_delta*ratio)])
        current=poses[-1].copy()
    angles=np.linspace(-math.pi,math.pi,720)
    odom=poses[0].copy();previous=poses[0]
    with path.open('w') as f:
        f.write(f'{grid.shape[1]} {grid.shape[0]} {meta["resolution"]} {meta["origin"][0]} {meta["origin"][1]}\n')
        occupied=np.where(grid<100,1,np.where(grid>250,-1,0));np.savetxt(f,occupied,fmt='%d')
        f.write(f'{len(poses)} {len(angles)}\n')
        for i,pose in enumerate(poses):
            delta=pose[:2]-previous[:2];c,s=math.cos(previous[2]),math.sin(previous[2])
            body=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
            body=body*np.array([1.06,.88])+rng.normal(0,.0005,2)
            c,s=math.cos(odom[2]),math.sin(odom[2]);odom[:2]+=[c*body[0]-s*body[1],s*body[0]+c*body[1]]
            odom[2]=wrap(odom[2]+wrap(pose[2]-previous[2])*1.04+rng.normal(0,.0008))
            direction=np.array([np.cos(angles+pose[2]),np.sin(angles+pose[2])])
            ranges=np.full(len(angles),8.)
            for box,dims in boxes:
                local=box[:2,:2].T.dot(pose[:2]-box[:2,3]);rays=box[:2,:2].T.dot(direction)
                with np.errstate(divide='ignore',invalid='ignore'):
                    a=(-dims[:2,None]/2-local[:,None])/rays;b=(dims[:2,None]/2-local[:,None])/rays
                enter=np.max(np.minimum(a,b),axis=0);leave=np.min(np.maximum(a,b),axis=0)
                hit=(leave>=np.maximum(enter,0))&(enter>=.08)
                ranges[hit]=np.minimum(ranges[hit],enter[hit])
            ranges=np.clip(ranges+rng.normal(0,.008,len(angles)),.08,8)
            ranges[rng.rand(len(angles))<.02]=8.
            f.write(' '.join(map(str,[i/6,*pose,*odom,*ranges]))+'\n');previous=pose
    return len(poses)


def metrics(path):
    data=np.genfromtxt(path,delimiter=',',names=True)
    valid=data['time']>=5
    result={}
    for source in ('odom','est'):
        error=np.hypot(data[source+'_x']-data['true_x'],data[source+'_y']-data['true_y'])[valid]
        yaw=np.abs(wrap(data[source+'_yaw']-data['true_yaw']))[valid]
        result[source]={'rmse_m':float(np.sqrt(np.mean(error**2))),'p95_m':float(np.percentile(error,95)),
                        'yaw_rmse_deg':float(np.degrees(np.sqrt(np.mean(yaw**2)))),
                        'max_error_m':float(error.max())}
    result['update_elapsed_mean_ms']=float(data['update_ms'].mean())
    result['update_elapsed_p95_ms']=float(np.percentile(data['update_ms'],95))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT.parent/'doc/validation/localization_core')
    parser.add_argument('--seeds',type=int,nargs='+',default=[11,22,33])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report={'kind':'installed AMCL C++ core; synthetic raycast scans and injected odometry errors',
            'timing_clock':'std::chrono::steady_clock elapsed time around the core update; not process CPU time',
            'world_sha256':hashlib.sha256((ROOT/'worlds/smart_community.world').read_bytes()).hexdigest(),
            'map_sha256':hashlib.sha256((ROOT/'maps/field_reference.pgm').read_bytes()).hexdigest(),
            'libraries':['libamcl_sensors.so','libamcl_pf.so','libamcl_map.so'],
            'not_tested':['ROS/Gazebo execution','Cartographer','DWA','TEB','real laser data'],
            'scenario':{'sample_rate_hz':6,'laser_noise_sigma_m':.008,'dropout':.02,'forward_odom_scale':1.06,'lateral_odom_scale':.88,
                        'angular_odom_scale':1.04,'initial_pose_error':[.08,-.06,.06],'evaluation_starts_after_s':5},'runs':[]}
    with tempfile.TemporaryDirectory(prefix='community_amcl_') as folder:
        folder=Path(folder);binary=folder/'benchmark'
        subprocess.run(['g++','-O2','-std=c++14',str(Path(__file__).with_name('amcl_core_benchmark.cpp')),
            '-I/opt/ros/noetic/include','-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
            '-lamcl_sensors','-lamcl_pf','-lamcl_map','-o',str(binary)],check=True)
        for seed in args.seeds:
            source=folder/f'input_{seed}.txt';frames=dataset(source,seed)
            for name,beams,minimum,maximum in [('amcl_baseline',90,300,1500),('amcl_dense',180,500,2000)]:
                out=args.output/f'{name}_{seed}.csv';begin=time.monotonic()
                subprocess.run([str(binary),str(source),str(out),str(seed),str(beams),str(minimum),str(maximum)],check=True)
                row=dict(algorithm=name,seed=seed,frames=frames,wall_seconds=time.monotonic()-begin,**metrics(out))
                report['runs'].append(row);print(json.dumps(row),flush=True)
    report['aggregate_mean_over_seeds']={}
    for algorithm in ('amcl_baseline','amcl_dense'):
        rows=[r for r in report['runs'] if r['algorithm']==algorithm]
        aggregate={key:float(np.mean([r['est'][key] for r in rows])) for key in ('rmse_m','p95_m','yaw_rmse_deg')}
        aggregate['mean_core_elapsed_ms']=float(np.mean([r['update_elapsed_mean_ms'] for r in rows]))
        report['aggregate_mean_over_seeds'][algorithm]=aggregate
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
