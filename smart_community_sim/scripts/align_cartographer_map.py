#!/usr/bin/env python3
"""Transform an exported Cartographer PGM/YAML into the competition's fixed map frame."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


def align(source_path,metadata_path,output_path):
    source=yaml.safe_load(source_path.read_text());metadata=json.loads(metadata_path.read_text())
    pixels=np.flipud(np.asarray(Image.open(source_path.parent/source['image']).convert('L')))
    h,w=pixels.shape;resolution=float(source['resolution'])
    if not math.isfinite(resolution) or resolution<=0 or source.get('negate',0)!=0:
        raise ValueError('Require positive resolution and an ordinary, non-negated Cartographer map')
    ax,ay,angle=map(float,metadata['anchor']);ox,oy,yaw=map(float,source['origin'])
    rotation=lambda a:np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    r=rotation(angle+yaw);origin=np.array([ax,ay])+rotation(angle).dot([ox,oy])
    corners=origin[:,None]+r.dot(np.array([[0,w*resolution,w*resolution,0],[0,0,h*resolution,h*resolution]]))
    low=np.floor(corners.min(axis=1)/resolution+1e-8)*resolution
    high=np.ceil(corners.max(axis=1)/resolution-1e-8)*resolution
    width,height=np.rint((high-low)/resolution).astype(int)
    xx,yy=np.meshgrid(low[0]+(np.arange(width)+.5)*resolution,low[1]+(np.arange(height)+.5)*resolution)
    local=r.T.dot(np.vstack((xx.ravel()-origin[0],yy.ravel()-origin[1])))
    ix,iy=np.floor(local/resolution).astype(int);valid=(ix>=0)&(iy>=0)&(ix<w)&(iy<h)
    result=np.full(width*height,205,np.uint8);result[valid]=pixels[iy[valid],ix[valid]]
    output_path=output_path.with_suffix('.yaml');image_path=output_path.with_suffix('.pgm')
    if output_path.exists() or image_path.exists():raise ValueError('Output exists; select another filename')
    output_path.parent.mkdir(parents=True,exist_ok=True)
    Image.fromarray(np.flipud(result.reshape(height,width))).save(image_path)
    target=dict(source,image=image_path.name,origin=[float(low[0]),float(low[1]),0.])
    output_path.write_text(yaml.safe_dump(target,sort_keys=False))
    return target


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('native_map',type=Path);parser.add_argument('state_metadata',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();align(args.native_map,args.state_metadata,args.output)
    print('Aligned map saved. Generate an aligned keepout map before navigation.')

if __name__=='__main__':main()
