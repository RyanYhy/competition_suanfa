#!/usr/bin/env python3
"""Check the actual expanded URDF geometry through one hub revolution (offline)."""
import json
import math
import sys
import xml.etree.ElementTree as E
r=E.parse(sys.argv[1]).getroot()
links={x.get('name'):x for x in r.findall('link')}
joints={x.get('name'):x for x in r.findall('joint')}

def numbers(s):return [float(a) for a in s.split()]
def rotate(p,rpy):
    x,y,z=p;a,b,c=rpy
    y,z=math.cos(a)*y-math.sin(a)*z,math.sin(a)*y+math.cos(a)*z
    x,z=math.cos(b)*x+math.sin(b)*z,-math.sin(b)*x+math.cos(b)*z
    return (math.cos(c)*x-math.sin(c)*y,math.sin(c)*x+math.cos(c)*y,z)
base_z=numbers(joints['base_fixed_joint'].find('origin').get('xyz'))[2]
report={}
for wheel in ('front_left','front_right','rear_left','rear_right'):
    wj=joints[wheel+'_wheel_joint'];wheel_z=base_z+numbers(wj.find('origin').get('xyz'))[2]
    rollers=[j for n,j in joints.items() if n.startswith(wheel+'_roller_')]
    assert len(rollers)==16 and all(j.get('type')=='continuous' for j in rollers)
    surfaces=[]
    for j in rollers:
        origin=j.find('origin');pos=numbers(origin.get('xyz'));angles=numbers(origin.get('rpy'))
        link=links[j.find('child').get('link')]
        assert link.find('inertial/mass') is not None
        for col in link.findall('collision'):
            local=col.find('origin');offset=numbers(local.get('xyz')) if local is not None else (0,0,0)
            rotated=rotate(offset,angles)
            center=tuple(a+b for a,b in zip(pos,rotated));rad=float(col.find('geometry/sphere').get('radius'))
            surfaces.append((center,rad))
    clearances=[]
    for k in range(128):
        a=k*2*math.pi/128
        clearances.append(min(wheel_z+rotate(p,(0,a,0))[2]-rad for p,rad in surfaces))
    assert min(clearances)>-.00001 and max(clearances)<.0015,(wheel,min(clearances),max(clearances))
    hub=links[wheel+'_wheel_link'];hub_radius=float(hub.find('collision/geometry/cylinder').get('radius'))
    assert wheel_z-hub_radius>.02,'Hub rather than rollers would touch ground'
    j=joints[wheel+'_roller_12_joint'];axle=rotate((0,0,1),numbers(j.find('origin').get('rpy')))
    ratio=axle[1]/axle[0]
    assert abs(ratio-(-1 if wheel in ('front_left','rear_right') else 1))<1e-6
    report[wheel]={'passive_rollers':16,'ground_clearance_range_m':[min(clearances),max(clearances)],'hub_clearance_m':wheel_z-hub_radius,'bottom_axle_y_over_x':ratio}
print(json.dumps(report,indent=2))
