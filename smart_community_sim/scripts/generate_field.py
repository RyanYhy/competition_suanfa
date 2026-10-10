#!/usr/bin/env python3
"""Build the field using the September 24 rules and supplied scene textures."""
from pathlib import Path
import math
import json
import shutil
import hashlib
import xml.etree.ElementTree as E
from xml.dom import minidom
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
FIELD_HALF=2.1
WALL_THICKNESS=.005
WALL_HEIGHT=.50
GANTRY_HALF_SPAN=.3075
SIGNAL_HEIGHT=.41
ASSETS=ROOT.parent/"doc/复赛资料"
C={"road":"0.015 0.015 0.015 1", "white":"0.96 0.96 0.93 1", "black":"0.035 0.04 0.05 1",
   "grass":"0.77 0.84 0.73 1", "blue":"0.08 0.36 0.67 1", "teal":"0.10 0.62 0.63 1",
   "red":"0.85 0.06 0.045 1", "green":"0.12 0.78 0.12 1", "yellow":"0.98 0.68 0.04 1",
   "gray":"0.4 0.44 0.46 1", "silver":"0.72 0.75 0.78 1", "off":"0.10 0.12 0.12 1"}
for directory in ('worlds','materials/scripts','materials/textures','meshes','config'):
    (ROOT/directory).mkdir(parents=True,exist_ok=True)
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if not Path(FONT).exists(): FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
MATERIALS=[]

def add(p,t,text=None,**attrs):
    q=E.SubElement(p,t,attrs)
    if text is not None:q.text=str(text)
    return q

def seq(v):return ' '.join(f'{a:.6g}' for a in v)

def text(draw,xy,label,size=30,fill='black'):
    draw.text(xy,label,font=ImageFont.truetype(FONT,size),fill=fill)

def texture(name,paint,size=(512,512),bg='#f2f1e8'):
    im=Image.new('RGB',size,bg); d=ImageDraw.Draw(im); paint(d)
    im.save(ROOT/'materials/textures'/f'{name}.png')
    MATERIALS.append(f'material Community/{name}\n{{\n technique {{ pass {{ ambient 1 1 1 1\n diffuse 1 1 1 1\n texture_unit {{ texture {name}.png }} }} }}\n}}')

def label(name,title,bg='#f2f1e8',fg='black'):
    texture(name,lambda d:text(d,(16,18),title,48,fg),(512,100),bg)

# Front face normal is +X. Viewed from +X with +Z up, screen-right is +Y:
# U must increase with Y. Reversing U mirrors every shared text/plate texture.
(ROOT/'meshes/sign.dae').write_text('''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset><library_geometries><geometry id="sign"><mesh><source id="p"><float_array id="pa" count="12">0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5</float_array><technique_common><accessor source="#pa" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><source id="uv"><float_array id="uva" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uva" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="2"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#sign"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')
# Ellipsoidal visual for the passive roller; collision uses three tapered spheres.
verts=[]; indices=[]
for i in range(13):
    a=math.pi*i/12
    for j in range(16):
        b=2*math.pi*j/16
        verts.extend((.014*math.sin(a)*math.cos(b),.014*math.sin(a)*math.sin(b),.025*math.cos(a)))
for i in range(12):
    for j in range(16):
        a=i*16+j;b=i*16+(j+1)%16;c=(i+1)*16+j;d=(i+1)*16+(j+1)%16
        indices.extend((a,c,b,b,c,d))
(ROOT/'meshes/mecanum_roller.dae').write_text(f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset><library_geometries><geometry id="roller"><mesh><source id="p"><float_array id="pa" count="{len(verts)}">{seq(verts)}</float_array><technique_common><accessor source="#pa" count="{len(verts)//3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="{len(indices)//3}"><input semantic="VERTEX" source="#v" offset="0"/><p>{' '.join(map(str,indices))}</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#roller"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')

sdf=E.Element('sdf',version='1.6'); world=add(sdf,'world',name='smart_community')
add(world,'gravity','0 0 -9.81')
physics=add(world,'physics',name='contact_physics',type='ode')
# Live contact/heading sweeps: 800 iterations reduce constraint drift; a 4 ms
# step maintains near-real-time simulation with all 64 passive rollers.
add(physics,'max_step_size',.004);add(physics,'real_time_update_rate',250)
ode=add(physics,'ode'); solver=add(ode,'solver');add(solver,'type','quick');add(solver,'iters',800);add(solver,'sor',1.0)
constraints=add(ode,'constraints');add(constraints,'cfm',0);add(constraints,'erp',.2)
add(constraints,'contact_max_correcting_vel',.1);add(constraints,'contact_surface_layer',.0001)
light=add(world,'light',name='sun',type='directional');add(light,'pose','0 0 8 0 0 0');add(light,'diffuse','0.8 0.8 0.8 1');add(light,'direction','-0.3 0.2 -1')
scene=add(world,'scene');add(scene,'ambient','0.65 0.65 0.65 1');add(scene,'background','0.80 0.85 0.9 1');add(scene,'shadows','true')
cam=add(add(world,'gui'),'camera',name='overview');add(cam,'pose','-4.8 5.6 6.4 0 0.72 -0.862')

def model(name,x=0,y=0,z=0,yaw=0):
    m=add(world,'model',name=name);add(m,'static','true');add(m,'pose',seq((x,y,z,0,0,yaw)))
    return m,add(m,'link',name='body')

def part(link,name,shape,dims,pose=(0,0,0,0,0,0),color='white',solid=True,tex=None):
    for tag in (('visual','collision') if solid else ('visual',)):
        el=add(link,tag,name=name);add(el,'pose',seq(pose));g=add(el,'geometry');g=add(g,shape)
        if shape=='box':add(g,'size',seq(dims))
        elif shape=='cylinder':add(g,'radius',dims[0]);add(g,'length',dims[1])
        elif shape=='sphere':add(g,'radius',dims[0])
        elif shape=='mesh':add(g,'uri','model://smart_community_sim/meshes/sign.dae');add(g,'scale',seq((1,*dims)))
        if tag=='visual':
            mat=add(el,'material')
            if tex:
                sc=add(mat,'script');add(sc,'uri','model://smart_community_sim/materials/scripts');add(sc,'uri','model://smart_community_sim/materials/textures');add(sc,'name','Community/'+tex)
            else:add(mat,'ambient',C[color]);add(mat,'diffuse',C[color])
        else:
            fr=add(add(add(el,'surface'),'friction'),'ode');add(fr,'mu',1.0);add(fr,'mu2',1.0)

def box(l,n,x,y,z,sx,sy,sz,c='white',solid=True,rpy=(0,0,0)):
    part(l,n,'box',(sx,sy,sz),(x,y,z,*rpy),c,solid)

def cylinder(l,n,x,y,z,r,h,c='gray',rpy=(0,0,0),solid=True):
    part(l,n,'cylinder',(r,h),(x,y,z,*rpy),c,solid)

def board(l,n,x,y,z,w,h,tex,yaw=0):
    part(l,n,'mesh',(w,h),(x,y,z,0,0,yaw),solid=False,tex=tex)

def beam(l,n,a,b,width,c):
    d=[v-u for u,v in zip(a,b)];length=math.sqrt(sum(v*v for v in d))
    box(l,n,*[(v+u)/2 for u,v in zip(a,b)],length,width,width,c,False,(0,-math.atan2(d[2],math.hypot(*d[:2])),math.atan2(d[1],d[0])))

_,floor=model('field_floor');box(floor,'ground',0,0,-.05,
    2*FIELD_HALF,2*FIELD_HALF,.1,'road')
contact=add(floor,'sensor',name='floor_contacts',type='contact')
add(contact,'always_on','true');add(contact,'update_rate',50)
add(add(contact,'contact'),'collision','ground')
cp=add(contact,'plugin',name='ground_contact_ros',filename='libgazebo_ros_bumper.so')
add(cp,'bumperTopicName','/ground_contacts');add(cp,'frameName','world')
# Wall inner faces stay at +/-2.1 m so the existing 0.60 m roads retain their width.
_,walls=model('perimeter_walls')
wall_center=FIELD_HALF+WALL_THICKNESS/2
for name,x,y,sx,sy in [
    ('north',0,wall_center,2*(FIELD_HALF+WALL_THICKNESS),WALL_THICKNESS),
    ('south',0,-wall_center,2*(FIELD_HALF+WALL_THICKNESS),WALL_THICKNESS),
    ('east',wall_center,0,WALL_THICKNESS,2*FIELD_HALF),
    ('west',-wall_center,0,WALL_THICKNESS,2*FIELD_HALF)]:
    box(walls,name,x,y,WALL_HEIGHT/2,sx,sy,WALL_HEIGHT,'white')
_,paint=model('painted_boundaries')
# All labelled dimensions are 0.60 m: right road, bottom road and A-zone depth.
regions=[('A街区',-1.5,.9,.8,1.5),('中央禁行区',.2,-1.5,.8,.9),('B街区及预留区',-1.5,-1.5,-.4,.3),('停车区',1.4,-2.1,2.1,1.5)]
for i,(_,x1,y1,x2,y2) in enumerate(regions):
    # Invisible regions are consumed by the independent keepout-map generator.
    box(paint,'island_'+str(i),(x1+x2)/2,(y1+y2)/2,.001,x2-x1,y2-y1,.001,'road',False)
# Boundaries are paint; the diagram does not specify tall interior walls.
segments=[(-2.1,-2.1,2.1,-2.1),(2.1,-2.1,2.1,2.1),(2.1,2.1,-2.1,2.1),(-2.1,2.1,-2.1,-2.1),
          (-1.5,1.5,.8,1.5),(-1.5,.9,.2,.9),(-1.5,.9,-1.5,1.5),(.8,-1.5,.8,1.5),(.2,-1.5,.2,.9),(.2,-1.5,.8,-1.5),
          (-1.5,-1.5,-.4,-1.5),(-1.5,.3,-.4,.3),(-1.5,-1.5,-1.5,.3),(-.4,-1.5,-.4,.3),(1.4,-2.1,1.4,2.1),(-1.5,-.55,-.4,-.55)]
for i,(a,b,c,d) in enumerate(segments):
    box(paint,'edge_'+str(i),(a+c)/2,(b+d)/2,.005,math.hypot(c-a,d-b),.014,.005,'white',False,(0,0,math.atan2(d-b,c-a)))
# Zebra crossing stripes run parallel to traffic, with stop lines on the approach.
for i in range(7):
    box(paint,'north_zebra_'+str(i),-.02,1.545+i*.079,.008,.21,.035,.004,'white',False)
    box(paint,'south_zebra_'+str(i),-.355+i*.079,-.70,.008,.035,.21,.004,'white',False)
box(paint,'north_stop',.30,1.80,.008,.025,.56,.004,'white',False)
box(paint,'south_stop',-.10,-.55,.008,.56,.025,.004,'white',False)
box(paint,'start_line',1.38,1.8,.008,.025,.56,.004,'white',False)
for i,y in enumerate((-1.5,-.9,-.3,1.5)):
    box(paint,'bay_line_'+str(i),1.75,y,.007,.7,.018,.004,'white',False)


# Copy source files byte-for-byte. Runtime does not need doc/.
asset_records=[]
def bundle_texture(name,source):
    destination=ROOT/'materials/textures'/f'{name}.png'
    # A source release already contains byte-identical runtime textures.
    # Reuse them when the organizer's original doc folder is not distributed.
    if source.is_file():
        if source.resolve()!=destination.resolve():shutil.copyfile(source,destination)
    elif not destination.is_file():
        raise FileNotFoundError(f'Missing original and bundled texture: {source}; {destination}')
    MATERIALS.append(f"""material Community/{name}
{{
 technique {{ pass {{ lighting off
 scene_blend alpha_blend
 alpha_rejection greater 128
 cull_hardware clockwise
 texture_unit {{ texture {name}.png }} }} }}
}}""")

def official(name,relative,category=None,label=None):
    bundle_texture(name,ASSETS/relative)
    if category:
        asset_records.append(dict(id=name,category=category,label=label,
                                  texture=f'materials/textures/{name}.png',source=relative,
                                  sha256=hashlib.sha256((ROOT/'materials/textures'/f'{name}.png').read_bytes()).hexdigest()))
    return name

for i in range(1,17):
    official(f'resident_{i}',f'人员/社区人员/{i}.png','person',f'社区人员{i}')
for i in range(1,3):
    official(f'visitor_F{i}',f'人员/非社区人员/F{i}.png','person',f'非社区人员F{i}')
for col,cn in [('red','红'),('yellow','黄'),('green','绿')]:
    for state,desc in [('on','亮'),('off','暗')]:
        official(f'signal_{col}_{state}',f'红绿灯/红绿灯图片/{cn}灯{desc}.png')
plates=['苏A·B8Q62','鄂D·7B5Q2','苏A·PL12A']
for i,cn in enumerate(('一','二','三'),1):
    official(f'plate_{i}',f'车辆识别/车牌{cn}.png','plate',plates[i-1])
official('car_background','车辆识别/车牌背景.png')
for i in (1,2):
    # Scene-only assets: do not teach the legacy recognizer these random plates.
    bundle_texture(f'random_plate_{i}',ROOT.parent/'doc'/f'随机车牌{i}')
scene_plates=[
    dict(model='license_plate_1',texture='random_plate_1',source='doc/随机车牌1',kind='user_random'),
    dict(model='license_plate_2',texture='random_plate_2',source='doc/随机车牌2',kind='user_random'),
    dict(model='license_plate_3',texture='plate_3',source='doc/复赛资料/车辆识别/车牌三.png',kind='official_example'),
]

# PDF p7: full-height posts flank the 0.59 m housing, overall width 0.64 m.
gantries=[('north_signal',-.47,1.80,0),('south_signal',-.10,-1.43,math.pi/2)]
for name,x,y,yaw in gantries:
    m,l=model(name,x,y,yaw=yaw)
    for side,v in [('left',-GANTRY_HALF_SPAN),('right',GANTRY_HALF_SPAN)]:
        box(l,side+'_post',0,v,.24,.025,.025,.48,'black')
    box(l,'housing',0,0,SIGNAL_HEIGHT,.05,.59,.14,'black')
    # Viewed from the approaching robot, red is on the left.
    for col,v in [('red',-.22),('yellow',0),('green',.22)]:
        board(l,col,.025001,v,SIGNAL_HEIGHT,.12,.12,
              f'signal_{col}_'+('on' if col=='red' else 'off'))
    plugin=add(m,'plugin',name='signal_cycle',filename='libcommunity_signal.so')
    add(plugin,'redSeconds',10);add(plugin,'greenSeconds',15);add(plugin,'yellowSeconds',5)

# All 16 residents and 2 visitors are placed, one independent model per person.
# Compact, separate rows per viewing direction; 9-10 cm center spacing leaves
# 4-5 cm between 5 cm boards. Each row fits one camera observation without occlusion.
# Arrow directions: A west/south/north; B north/east.
persons=[
    ('A','resident_1',-1.36,1.17,math.pi),
    ('A','resident_2',-1.10,1.07,-math.pi/2),
    ('A','visitor_F1',-1.00,1.07,-math.pi/2),
    ('A','resident_3',-.90,1.07,-math.pi/2),
    ('A','resident_4',-.85,1.32,math.pi/2),
    ('A','resident_9',-.95,1.32,math.pi/2),
    ('A','resident_10',-.75,1.32,math.pi/2),
    ('A','resident_11',-.80,1.07,-math.pi/2),
    ('A','resident_12',-1.36,1.27,math.pi),
    ('B','resident_5',-1.15,.14,math.pi/2),
    ('B','resident_6',-.95,.14,math.pi/2),
    ('B','resident_7',-.85,.14,math.pi/2),
    ('B','visitor_F2',-.53,-.08,0),
    ('B','resident_8',-.53,-.28,0),
    ('B','resident_13',-1.05,.14,math.pi/2),
    ('B','resident_14',-.75,.14,math.pi/2),
    ('B','resident_15',-.53,-.18,0),
    ('B','resident_16',-.53,-.38,0),
]
for district,identity,x,y,yaw in persons:
    height=.15; width=.05
    with Image.open(ROOT/'materials/textures'/f'{identity}.png') as im:
        artwork_scale=min(width/im.width,height/im.height)
        artwork_width=im.width*artwork_scale;artwork_height=im.height*artwork_scale
    _,l=model(f'person_{district}_{identity}',x,y,yaw=yaw)
    box(l,'standee',0,0,height/2,.005,width,height,'white')
    board(l,'portrait',.002501,0,height/2,artwork_width,artwork_height,identity)

for i,y in enumerate((-1.79,-1.19,-.59),1):
    _,l=model(f'car_{i}',1.77,y,yaw=math.pi)
    box(l,'backing',0,0,.125,.0005,.345,.25,'white')
    board(l,'car_image',.000251,0,.125,.345,.25,'car_background')
    # Independent static model mounted on the front of the background board.
    # Local +X faces west; a 10 um gap separates the two collision surfaces.
    _,plate_link=model(f'license_plate_{i}',1.77-.00031,y,z=.125,yaw=math.pi)
    box(plate_link,'standee',0,0,0,.0001,.095,.03,'white')
    board(plate_link,'number_plate',.000051,0,0,.095,.03,scene_plates[i-1]['texture'])

(ROOT/'materials/scripts/smart_community.material').write_text('\n'.join(MATERIALS))
(ROOT/'worlds/smart_community.world').write_text(minidom.parseString(E.tostring(sdf)).toprettyxml(indent='  '))
import copy
test=E.Element('sdf',version='1.6');tw=add(test,'world',name='drive_test')
for item in (physics,light,scene):tw.append(copy.deepcopy(item))
f=copy.deepcopy(world.find("model[@name='field_floor']"))
f.find('.//box/size').text='6 6 0.1';f.find('.//collision/geometry/box/size').text='6 6 0.1';tw.append(f)
(ROOT/'worlds/drive_test.world').write_text(minidom.parseString(E.tostring(test)).toprettyxml(indent='  '))

# Schematic preview generated directly from geometry; not a Gazebo screenshot.
im=Image.new('RGB',(1000,1040),'white');d=ImageDraw.Draw(im)
def pt(x,y):return (int(70+(x+2.1)*200),int(70+(2.1-y)*200))
d.rectangle((*pt(-2.1,2.1),*pt(2.1,-2.1)),fill='#101010',outline='#888888',width=4)
for a,b,c,e in segments:d.line((*pt(a,b),*pt(c,e)),fill='white',width=3)
for i,y in enumerate((-1.5,-.9,-.3,1.5)):
    d.line((*pt(1.4,y),*pt(2.1,y)),fill='white',width=3)
for i in range(7):
    d.rectangle((*pt(-.125,1.562+i*.079),*pt(.085,1.527+i*.079)),fill='white')
    d.rectangle((*pt(-.372+i*.079,-.595),*pt(-.337+i*.079,-.805)),fill='white')
for a,b in [((.30,1.52),(.30,2.08)),((-.38,-.55),(.18,-.55))]:d.line((*pt(*a),*pt(*b)),fill='white',width=5)
for district,identity,x,y,yaw in persons:
    a,b=pt(x,y);d.ellipse((a-6,b-6,a+6,b+6),fill='#ee6677' if identity.startswith('visitor') else '#f1b845')
    d.line((a,b,*pt(x+.12*math.cos(yaw),y+.12*math.sin(yaw))),fill='orange',width=3)
for i,y in enumerate((-1.79,-1.19,-.59),1):
    d.line((*pt(1.77,y+.1725),*pt(1.77,y-.1725)),fill='#5599ee',width=5)
    text(d,pt(1.46,y+.25),str(i)+'号 '+('随机牌' if i<3 else '示例牌'),15,'white')
for name,x,y,yaw in gantries:
    d.line((*pt(x+.32*math.sin(yaw),y-.32*math.cos(yaw)),
            *pt(x-.32*math.sin(yaw),y+.32*math.cos(yaw))),fill='#888888',width=7)
route=[(1.65,1.8),(-1.8,1.8),(-1.8,.6),(-.1,.6),(-.1,-1.8),(1.1,-1.8),(1.1,1.8),(1.65,1.8)]
for a,b in zip(route,route[1:]):
    d.line((*pt(*a),*pt(*b)),fill='#558fcd',width=3)
    dx,dy=b[0]-a[0],b[1]-a[1];norm=math.hypot(dx,dy);ux,uy=dx/norm,dy/norm
    tip=((a[0]+b[0])/2,(a[1]+b[1])/2)
    d.polygon([pt(*tip),pt(tip[0]-.09*ux+.04*uy,tip[1]-.09*uy-.04*ux),
               pt(tip[0]-.09*ux-.04*uy,tip[1]-.09*uy+.04*ux)],fill='#558fcd')
text(d,pt(1.43,2.02),'起终点',20,'white')
text(d,pt(-1.42,1.0),'A 区：8 居民 + 1 非居民',13,'white')
text(d,pt(-1.42,-.48),'B 区：8 居民 + 1 非居民',13,'white')
text(d,(70,937),'9.24 规则修订：4.2 × 4.2 m；18 人；2 随机牌 + 1 示例牌',21)
text(d,(70,978),'俯视示意，非 Gazebo 截图。黄色：居民；红色：非居民。',19)
im.save(ROOT/'field_layout.png')
manifest={'field_size_m':[4.2,4.2],'road_width_m':.6,
    'perimeter_wall':{'height_m':WALL_HEIGHT,'thickness_m':WALL_THICKNESS,'source':'9.24 PDF p14 required walls; p15 recommended dimensions'},
    'traffic_gantry':{'outer_width_m':.64,'post_width_m':.025,'post_center_span_m':.615,
                      'clear_width_m':.590,'total_height_m':.48,'housing_height_m':.14,'housing_width_m':.59,
                      'depth_m':.05,'post_height_m':.48,'clear_height_m':.34},
    'car_board_m':{'width':.345,'height':.25,'thickness':.0005},
    'plate_m':{'width':.095,'height':.03,'thickness':.0001,'independent_models':True},
    'person_board_m':{'width':.05,'height':.15,'thickness':.005},
    'person_height_m':.15,'person_height_source':'9.24 PDF p5; artwork fits without stretching',
    'source':'doc/9.24智慧社区详细规则.pdf pp3-8,14-15; original organizer textures and user random plate images',
    'persons':{'A':9,'B':9},'person_counts':{'residents':16,'visitors':2},
    'person_placements':[dict(district=a,identity=b,x=c,y=d,yaw=e) for a,b,c,d,e in persons],
    'plates':scene_plates,'signals_seconds':{'red':10,'green':15,'yellow':5},
    'assumptions':['three parking bays: two user random plates and one official example',
                   'map schematic normalized to labelled 0.60 m lanes; not a surveyed map',
                   'standee identities, within-district offsets and plate mounting height are simulation choices',
                   'PDF p5 person width 5 cm takes precedence over p19 tutorial example 4 cm'],
    'models':len(world.findall('model'))}
(ROOT/'config/scene_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
(ROOT/'config/recognition_assets.json').write_text(json.dumps(asset_records,ensure_ascii=False,indent=2)+'\n')
print('Generated official-material field, test floor, texture library and schematic preview.')
