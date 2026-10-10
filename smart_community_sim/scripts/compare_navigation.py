#!/usr/bin/env python3
"""Run isolated, sequential Gazebo comparisons; write missing/blocked results explicitly."""
import argparse
import itertools
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rospkg
import roslib.packages


def free_port():
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1',0));return sock.getsockname()[1]


def preflight():
    packages={}
    for name in ('amcl','teb_local_planner','dwa_local_planner','move_base','gazebo_ros','cartographer_ros','cartographer_ros_msgs','smart_community_sim'):
        try:packages[name]=rospkg.RosPack().get_path(name)
        except rospkg.ResourceNotFound:packages[name]=None
    try:free_port();network=dict(allowed=True,reason='')
    except OSError as exc:network=dict(allowed=False,reason=str(exc))
    executables={}
    for package,node in [('amcl','amcl'),('move_base','move_base'),('gazebo_ros','gzserver'),
                         ('cartographer_ros','cartographer_node'),('smart_community_sim','navigation_benchmark.py')]:
        try:executables[package+'/'+node]=bool(roslib.packages.find_node(package,node))
        except rospkg.ResourceNotFound:executables[package+'/'+node]=False
    return dict(packages=packages,executables=executables,local_ros_network=network)


def stop(process):
    if process.poll() is not None:return
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--localizers',nargs='+',choices=['amcl','amcl_dense','cartographer'],default=['amcl','amcl_dense','cartographer'])
    parser.add_argument('--planners',nargs='+',choices=['dwa','teb'],default=['dwa','teb'])
    parser.add_argument('--profiles',nargs='+',choices=['baseline','balanced','fast'],default=['balanced'])
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--state',type=Path)
    parser.add_argument('--map',type=Path,help='Common map in competition coordinates')
    parser.add_argument('--keepout',type=Path,help='Keepout YAML aligned to the common map')
    parser.add_argument('--output',type=Path,default=Path('/tmp/community_comparison')/time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout',type=float,default=900)
    parser.add_argument('--check',action='store_true',help='Check dependencies and permissions only')
    args=parser.parse_args()
    if bool(args.map)!=bool(args.keepout):parser.error('--map and --keepout must be provided together')
    if args.repeats<1:parser.error('--repeats must be positive')
    if args.timeout<=0:parser.error('--timeout must be positive')
    for path in (args.map,args.keepout):
        if path and not path.expanduser().is_file():parser.error('Map does not exist: '+str(path))
    root=args.output.expanduser().resolve();root.mkdir(parents=True,exist_ok=True)
    if (root/'summary.json').exists():parser.error('Choose a new output directory; existing reports are preserved')
    state=args.state.expanduser().resolve() if args.state else None
    report=dict(kind='online comparison availability' if args.check else 'online algorithm comparison',preflight=preflight(),runs=[])
    packages=report['preflight']['packages']
    report['state_available']=bool(state and state.is_file() and Path(str(state)+'.json').is_file())
    for localizer,planner,profile,repeat in itertools.product(args.localizers,args.planners,args.profiles,range(1,args.repeats+1)):
        row=dict(localization=localizer,planner=planner,profile=profile,repeat=repeat)
        missing=[]
        for dep in ['smart_community_sim','move_base','gazebo_ros']+(['cartographer_ros','cartographer_ros_msgs'] if localizer=='cartographer' else ['amcl'])+(['teb_local_planner'] if planner=='teb' else ['dwa_local_planner']):
            if not packages[dep]:missing.append(dep)
        for executable in ['move_base/move_base','gazebo_ros/gzserver','smart_community_sim/navigation_benchmark.py']+(['cartographer_ros/cartographer_node'] if localizer=='cartographer' else ['amcl/amcl']):
            if not report['preflight']['executables'][executable]:missing.append(executable+' executable')
        if missing:row.update(status='missing_dependencies',reason=', '.join(missing))
        elif localizer=='cartographer' and not report['state_available']:row.update(status='missing_map',reason='Need a scene-specific .pbstream and its .json metadata')
        elif not report['preflight']['local_ros_network']['allowed']:row.update(status='blocked',reason=report['preflight']['local_ros_network']['reason'])
        elif args.check:row.update(status='ready_not_run')
        else:
            folder=root/f'{localizer}_{planner}_{profile}_{repeat}';folder.mkdir()
            env=os.environ.copy();env.update(ROS_MASTER_URI=f'http://127.0.0.1:{free_port()}',
                GAZEBO_MASTER_URI=f'http://127.0.0.1:{free_port()}',ROS_IP='127.0.0.1',
                ROS_HOME=str(folder/'.ros'),ROS_LOG_DIR=str(folder/'roslog'))
            env.pop('ROS_HOSTNAME',None)
            command=['roslaunch','smart_community_sim','benchmark.launch',
                'localization:='+('amcl' if localizer.startswith('amcl') else 'cartographer'),
                'amcl_profile:='+('dense' if localizer=='amcl_dense' else 'baseline'),
                'planner:='+planner,'profile:='+profile,'output:='+str(folder),'wall_timeout:='+str(args.timeout)]
            if state:command.append('state:='+str(state))
            if args.map:
                command.extend(['map:='+str(args.map.expanduser().resolve()),'keepout_map:='+str(args.keepout.expanduser().resolve())])
            with (folder/'launch.log').open('w') as log:
                process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
                try:
                    process.wait(timeout=args.timeout+30)
                except subprocess.TimeoutExpired:row.update(status='timeout')
                finally:stop(process)
            result=folder/'report.json'
            if result.exists():row.update(status='measured',report=json.loads(result.read_text()),directory=str(folder))
            else:row.update(status=row.get('status','launch_failed'),directory=str(folder),returncode=process.returncode)
        report['runs'].append(row)
        (root/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:v for k,v in row.items() if k!='report'},ensure_ascii=False),flush=True)
    print('Summary: '+str(root/'summary.json'))
    if any(r['status'] not in ('measured','ready_not_run') for r in report['runs']):return 2
    return 0 if args.check or all(r['report']['passed_route'] and r['report']['accuracy_pass'] and r['report']['sufficient_samples'] for r in report['runs']) else 1

if __name__=='__main__':raise SystemExit(main())
