#!/usr/bin/env python3
"""Export expanded URDF inertias, primitive collisions and hinges to an ODE probe.
No ROS nodes, transport, rendering or Gazebo plugins are run by this diagnostic.
Massless fixed visual links are omitted; massive fixed sensor bodies are retained.
"""
import sys
import xml.etree.ElementTree as E
import numpy as np
from tf.transformations import euler_matrix, quaternion_from_matrix

def origin(element):
    result=np.eye(4)
    if element is not None:
        result=euler_matrix(*map(float,element.get('rpy','0 0 0').split()))
        result[:3,3]=list(map(float,element.get('xyz','0 0 0').split()))
    return result

def pose(matrix):
    q=quaternion_from_matrix(matrix)
    return [*matrix[:3,3],q[3],*q[:3]]

def export(source,output):
    root=E.parse(source).getroot();links={e.get('name'):e for e in root.findall('link')}
    joints={e.find('child').get('link'):e for e in root.findall('joint')}
    transforms={}
    def transform(name):
        if name not in transforms:
            if name not in joints:
                transforms[name]=np.eye(4);transforms[name][2,3]=.01
            else:
                joint=joints[name]
                transforms[name]=transform(joint.find('parent').get('link'))@origin(joint.find('origin'))
        return transforms[name]
    massive={name for name,link in links.items() if link.find('inertial') is not None}
    with open(output,'w') as stream:
        def write(*values):stream.write(' '.join(map(str,values))+'\n')
        for name in sorted(massive):
            link=links[name];inertia=link.find('inertial');i=inertia.find('inertia')
            assert np.allclose(origin(inertia.find('origin')),np.eye(4)), 'Probe requires COM at link origin'
            write('body',name,inertia.find('mass').get('value'),*[i.get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')],*pose(transform(name)))
            for collision in link.findall('collision'):
                shape=collision.find('geometry')[0];kind=shape.tag
                if kind=='sphere':dims=[float(shape.get('radius')),0,0]
                elif kind=='cylinder':dims=[float(shape.get('radius')),float(shape.get('length')),0]
                elif kind=='box':dims=list(map(float,shape.get('size').split()))
                else:raise ValueError(kind)
                write('shape',name,kind,*dims,*pose(origin(collision.find('origin'))))
        for child,joint in joints.items():
            if child not in massive:continue
            parent=joint.find('parent').get('link')
            while parent not in massive and parent in joints:parent=joints[parent].find('parent').get('link')
            if parent not in massive:continue
            matrix=transform(child);axis=joint.find('axis')
            vector=np.array(list(map(float,axis.get('xyz').split()))) if axis is not None else np.array([0,0,1])
            dynamics=joint.find('dynamics');damping=dynamics.get('damping',0) if dynamics is not None else 0
            write('joint',joint.get('name'),parent,child,joint.get('type'),*matrix[:3,3],*(matrix[:3,:3]@vector),damping)

if __name__=='__main__':export(*sys.argv[1:])
