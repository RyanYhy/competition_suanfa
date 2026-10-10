#!/usr/bin/env python3
"""Offline perspective projections, not Gazebo frames or navigation validation.

Supply expanded URDF. Check every placed target has a visible, recognizable
front face at a route observation. The camera and target geometry come from XML.
"""
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as E
import cv2
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'src'))
from generate_navigation_maps import transform
from smart_community_sim.vision_core import TemplateRecognizer, signal_state


def main():
    robot=E.parse(sys.argv[1]).getroot()
    camera=robot.find("gazebo[@reference='camera_link']/sensor/camera")
    width=int(camera.findtext('image/width'));height=int(camera.findtext('image/height'))
    focal=width/(2*math.tan(float(camera.findtext('horizontal_fov'))/2))
    origin=robot.find("joint[@name='camera_joint']/origin")
    camera_pose=E.Element('pose');camera_pose.text=origin.get('xyz')+' '+origin.get('rpy')
    camera_element=E.Element('camera');camera_element.append(camera_pose)
    camera_tf=transform(camera_element)
    camera_tf[2,3]+=float(robot.find("joint[@name='base_fixed_joint']/origin").get('xyz').split()[2])
    world=E.parse(str(ROOT/'worlds/smart_community.world')).getroot().find('world')
    route=yaml.safe_load((ROOT/'config/navigation/patrol_route.yaml').read_text())['waypoints']
    recognizer=TemplateRecognizer(ROOT)
    records=[];seen=set();expected=set();signal_checks=[];composite_seen=set();composites=[]
    for point in route:
        if not point['name'].startswith('observe_') and 'signal' not in point:continue
        vehicle=E.Element('model');E.SubElement(vehicle,'pose').text=f"{point['x']} {point['y']} 0 0 0 {point['yaw']}"
        camera_world=transform(vehicle).dot(camera_tf)
        to_camera=np.linalg.inv(camera_world)
        layers=[]
        for model in world.findall('model'):
            name=model.get('name')
            if 'district' in point and not name.startswith('person_'+point['district']+'_'):continue
            elif 'plate' in point['name'] and not name.startswith('car_'):continue
            elif 'signal' in point and name!=point['signal']:continue
            elif 'district' not in point and 'plate' not in point['name'] and 'signal' not in point:continue
            for visual in model.findall('link/visual'):
                if visual.get('name') not in ('portrait','number_plate','red','green','yellow'):continue
                material=visual.findtext('material/script/name','').split('/')[-1]
                is_signal=visual.get('name') in ('red','green','yellow')
                if not is_signal:expected.add(material)
                scale=visual.findtext('geometry/mesh/scale')
                if scale is None:continue
                _,w,h=map(float,scale.split())
                pose=transform(model).dot(transform(visual))
                if pose[:3,0].dot(camera_world[:3,3]-pose[:3,3])<=0:continue
                corners=np.array([[0,-w/2,h/2,1],[0,w/2,h/2,1],[0,w/2,-h/2,1],[0,-w/2,-h/2,1]]).T
                local=to_camera.dot(pose).dot(corners)
                if np.any(local[0]<=.05):continue
                quad=np.column_stack((width/2-focal*local[1]/local[0],height/2-focal*local[2]/local[0])).astype(np.float32)
                if np.any(quad[:,0]<0) or np.any(quad[:,0]>=width) or np.any(quad[:,1]<0) or np.any(quad[:,1]>=height):continue
                if is_signal:material='signal_'+visual.get('name')+'_on'
                source=cv2.imread(str(ROOT/'materials/textures'/(material+'.png')),cv2.IMREAD_UNCHANGED)
                if source.shape[2]==3:source=np.dstack((source,np.full(source.shape[:2],255,np.uint8)))
                sh,sw=source.shape[:2]
                mapping=cv2.getPerspectiveTransform(np.float32([[0,0],[sw-1,0],[sw-1,sh-1],[0,sh-1]]),quad)
                warped=cv2.warpPerspective(source,mapping,(width,height))
                layers.append((material,warped,mapping))
                alpha=warped[:,:,3:4].astype(float)/255
                frame=(warped[:,:,:3]*alpha+40*(1-alpha)).astype(np.uint8)
                results=recognizer.detect(frame,signals=is_signal)
                found=any(d['id']==material for d in results)
                if found:seen.add(material)
                records.append(dict(observation=point['name'],target=material,recognized=found,
                                    pixel_size=[round(float(v),1) for v in np.ptp(quad,axis=0)]))
                if is_signal:signal_checks.append(found)
        states=('red','yellow','green') if 'signal' in point else (None,)
        for state in states:
            composite=np.full((height,width,3),40,np.uint8)
            for material,layer,mapping in layers:
                if state and material!='signal_'+state+'_on':
                    off=cv2.imread(str(ROOT/'materials/textures'/(material.replace('_on','_off')+'.png')))
                    off=np.dstack((off,np.full(off.shape[:2],255,np.uint8)))
                    layer=cv2.warpPerspective(off,mapping,(width,height))
                alpha=layer[:,:,3:4].astype(float)/255
                composite=(layer[:,:,:3]*alpha+composite*(1-alpha)).astype(np.uint8)
            detected=recognizer.detect(composite,signals=bool(state))
            composite_seen.update(d['id'] for d in detected)
            observed=signal_state(detected) if state else None
            composites.append(dict(observation=point['name'],expected_signal=state,
                                   observed_signal=observed,ids=[d['id'] for d in detected]))
            if state:signal_checks.append(observed==state)
    report=dict(kind='synthetic geometry projection; not Gazebo rendering',
                missing=sorted(expected-seen),composite_missing=sorted(expected-composite_seen),
                records=records,composites=composites,signal_checks=signal_checks)
    target=ROOT.parent/'doc/validation/official_camera_projection.json'
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(missing=report['missing'],projections=len(records),composite_missing=report['composite_missing'],signals_pass=all(signal_checks)),ensure_ascii=False))
    return 0 if not report['missing'] and not report['composite_missing'] and len(signal_checks)==12 and all(signal_checks) else 1

if __name__=='__main__':sys.exit(main())
