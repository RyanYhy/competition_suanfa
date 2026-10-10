#!/usr/bin/env python3
"""Generate a lidar-height reference map and an independent painted-area keepout map.
This is scene-derived test data, not evidence of running SLAM.
"""
from pathlib import Path
import argparse
import math
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]
RESOLUTION=.02
ORIGIN=(-2.30,-2.30)
SIZE=230
LASER_HEIGHT=.265

def transform(element):
    values=[float(v) for v in element.findtext('pose','0 0 0 0 0 0').split()]
    x,y,z,roll,pitch,yaw=values
    a,b,c=math.cos(roll),math.cos(pitch),math.cos(yaw)
    d,e,f=math.sin(roll),math.sin(pitch),math.sin(yaw)
    t=np.eye(4)
    t[:3,:3]=[[c*b,c*e*d-f*a,c*e*a+f*d],
              [f*b,f*e*d+c*a,f*e*a-c*d],[-e,b*d,b*a]]
    t[:3,3]=(x,y,z)
    return t

def contains(geometry,pose,points,xy_padding=0.0):
    local=pose[:3,:3].T.dot(points-pose[:3,3,None])
    shape=next(iter(geometry))
    if shape.tag=='box':
        half=np.array([float(v) for v in shape.findtext('size').split()])[:,None]/2
        # Mark every cell touched by a thin wall, not only cells whose centres
        # happen to lie inside it. Project a world-XY half-cell into local axes.
        half += xy_padding*np.sum(np.abs(pose[:2,:3]),axis=0)[:,None]
        if xy_padding:
            # Sharing only a cell edge has zero overlap area. Including that
            # neighbour would thicken a 5 mm perimeter wall by another 2 cm.
            return np.all(np.abs(local)<half-1e-8,axis=0)
        return np.all(np.abs(local)<=half+1e-8,axis=0)
    if shape.tag=='sphere':return np.sum(local*local,axis=0)<=float(shape.findtext('radius'))**2
    if shape.tag=='cylinder':
        radius=float(shape.findtext('radius'));half=float(shape.findtext('length'))/2
        return (np.sum(local[:2]*local[:2],axis=0)<=radius**2)&(np.abs(local[2])<=half)
    raise ValueError('Unsupported collision geometry: '+shape.tag)

def write_map(name,pixels):
    target=ROOT/'maps';target.mkdir(exist_ok=True)
    # OccupancyGrid starts bottom-left, PGM starts top-left.
    Image.fromarray(np.flipud(pixels.reshape(SIZE,SIZE))).save(str(target/(name+'.pgm')))
    (target/(name+'.yaml')).write_text(yaml.safe_dump({
        'image':name+'.pgm','resolution':RESOLUTION,'origin':[*ORIGIN,0.0],
        'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))

def align_keepout(map_path, output):
    """Match both StaticLayer geometries so one cannot resize the other's grid."""
    metadata=yaml.safe_load(map_path.read_text())
    if abs(metadata['origin'][2])>1e-8:
        raise ValueError('Only axis-aligned maps are supported; map origin yaw must be zero.')
    with Image.open(map_path.parent/metadata['image']) as target:
        width,height=target.size
    source_meta=yaml.safe_load((ROOT/'maps/competition_keepout.yaml').read_text())
    source=np.flipud(np.array(Image.open(ROOT/'maps'/source_meta['image'])))
    resolution=float(metadata['resolution'])
    if not math.isfinite(resolution) or resolution<=0:raise ValueError('Invalid map resolution')
    xx,yy=np.meshgrid(metadata['origin'][0]+(np.arange(width)+.5)*resolution,
                     metadata['origin'][1]+(np.arange(height)+.5)*resolution)
    ix=np.floor((xx-source_meta['origin'][0])/source_meta['resolution']).astype(int)
    iy=np.floor((yy-source_meta['origin'][1])/source_meta['resolution']).astype(int)
    valid=(ix>=0)&(iy>=0)&(ix<source.shape[1])&(iy<source.shape[0])
    pixels=np.zeros((height,width),dtype=np.uint8)
    pixels[valid]=source[iy[valid],ix[valid]]
    output=output.with_suffix('.yaml');output.parent.mkdir(parents=True,exist_ok=True)
    image_path=output.with_suffix('.pgm')
    Image.fromarray(np.flipud(pixels)).save(str(image_path))
    output.write_text(yaml.safe_dump(dict(image=image_path.name,resolution=resolution,
        origin=metadata['origin'],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))
    print('Aligned keepout map: '+str(output))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--align-to-map',type=Path,help='Saved SLAM YAML in the scene-centred coordinate frame')
    parser.add_argument('--keepout-output',type=Path,help='Output aligned keepout YAML')
    args=parser.parse_args()
    if args.align_to_map:
        if not args.keepout_output:parser.error('--align-to-map requires --keepout-output')
        align_keepout(args.align_to_map,args.keepout_output)
        return
    if args.keepout_output:parser.error('--keepout-output requires --align-to-map')
    world=E.parse(str(ROOT/'worlds/smart_community.world')).getroot().find('world')
    xx,yy=np.meshgrid(ORIGIN[0]+(np.arange(SIZE)+.5)*RESOLUTION,
                       ORIGIN[1]+(np.arange(SIZE)+.5)*RESOLUTION)
    points=np.vstack((xx.ravel(),yy.ravel(),np.full(SIZE*SIZE,LASER_HEIGHT)))
    occupied=np.zeros(SIZE*SIZE,dtype=bool)
    for model in world.findall('model'):
        for link in model.findall('link'):
            base=transform(model).dot(transform(link))
            for collision in link.findall('collision'):
                occupied |= contains(collision.find('geometry'),base.dot(transform(collision)),points,RESOLUTION/2)
    outside=(np.abs(points[0])>2.17)|(np.abs(points[1])>2.17)
    pixels=np.full(SIZE*SIZE,254,dtype=np.uint8);pixels[outside]=205;pixels[occupied]=0
    write_map('field_reference',pixels)
    forbidden=(np.abs(points[0])>=2.1)|(np.abs(points[1])>=2.1)
    paint=world.find("model[@name='painted_boundaries']")
    for link in paint.findall('link'):
        base=transform(paint).dot(transform(link))
        for visual in link.findall('visual'):
            if not visual.get('name','').startswith('island_'):continue
            t=base.dot(transform(visual));ground=points.copy();ground[2]=t[2,3]
            # The white boundary paint extends 7 mm beyond the island centreline.
            import copy
            geometry=copy.deepcopy(visual.find('geometry'))
            size=[float(v) for v in geometry.findtext('box/size').split()]
            geometry.find('box/size').text=' '.join(map(str,(size[0]+.014,size[1]+.014,size[2])))
            forbidden |= contains(geometry,t,ground)
    mask=np.full(SIZE*SIZE,254,dtype=np.uint8);mask[forbidden]=0
    write_map('competition_keepout',mask)
    preview=np.repeat(np.flipud(pixels.reshape(SIZE,SIZE))[:,:,None],3,axis=2)
    preview[np.flipud(forbidden.reshape(SIZE,SIZE))]=(175,85,85)
    preview[np.flipud(occupied.reshape(SIZE,SIZE))]=(0,0,0)
    Image.fromarray(preview).resize((690,690),Image.NEAREST).save(str(ROOT/'maps/navigation_preview.png'))
    print('Generated reference lidar map and independent keepout map at 0.02 m/cell.')

if __name__=='__main__':main()
