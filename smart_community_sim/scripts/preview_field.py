#!/usr/bin/env python3
"""Render static SDF geometry/textures offline, without ROS or a Gazebo server.

These orthographic software previews are not Gazebo screenshots. They omit
lighting/shadows, sensors, robot dynamics and signal animation.
"""
from pathlib import Path
import itertools
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def transform(element):
    x, y, z, r, p, a = map(float, element.findtext('pose', '0 0 0 0 0 0').split())
    cr, cp, ca = np.cos([r, p, a]); sr, sp, sa = np.sin([r, p, a])
    m = np.eye(4)
    m[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                  [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = [x, y, z]
    return m


class Renderer:
    def __init__(self, width, height, eye, target, span):
        self.w, self.h = width, height
        self.eye = np.asarray(eye, dtype=float)
        self.target = np.asarray(target, dtype=float)
        backward = self.eye-self.target; backward /= np.linalg.norm(backward)
        right = np.cross([0, 0, 1], backward); right /= np.linalg.norm(right)
        up = np.cross(backward, right)
        self.basis = np.array([right, up, backward])
        self.scale = width/span
        self.rgb = np.full((height, width, 3), [230, 235, 241], dtype=np.uint8)
        self.depth = np.full((height, width), -np.inf)

    def project(self, vertices):
        p = (vertices-self.target).dot(self.basis.T)
        p[:, 0] = self.w/2+p[:, 0]*self.scale
        p[:, 1] = self.h/2-p[:, 1]*self.scale
        return p

    def triangle(self, vertices, color=None, texture=None, uv=None):
        v = self.project(vertices)
        lo = np.maximum(np.floor(v[:, :2].min(axis=0)).astype(int), [0, 0])
        hi = np.minimum(np.ceil(v[:, :2].max(axis=0)).astype(int), [self.w-1, self.h-1])
        if np.any(hi < lo): return
        xx, yy = np.meshgrid(np.arange(lo[0], hi[0]+1)+.5, np.arange(lo[1], hi[1]+1)+.5)
        den = (v[1, 1]-v[2, 1])*(v[0, 0]-v[2, 0])+(v[2, 0]-v[1, 0])*(v[0, 1]-v[2, 1])
        if abs(den) < 1e-10: return
        a = ((v[1, 1]-v[2, 1])*(xx-v[2, 0])+(v[2, 0]-v[1, 0])*(yy-v[2, 1]))/den
        b = ((v[2, 1]-v[0, 1])*(xx-v[2, 0])+(v[0, 0]-v[2, 0])*(yy-v[2, 1]))/den
        c = 1-a-b
        z = a*v[0, 2]+b*v[1, 2]+c*v[2, 2]
        region = np.s_[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
        mask = (a >= -1e-8) & (b >= -1e-8) & (c >= -1e-8) & (z > self.depth[region])
        if texture is None:
            self.rgb[region][mask] = color
        else:
            coords = a[..., None]*uv[0]+b[..., None]*uv[1]+c[..., None]*uv[2]
            tx = np.clip(np.rint(coords[..., 0]*(texture.shape[1]-1)).astype(int), 0, texture.shape[1]-1)
            ty = np.clip(np.rint(coords[..., 1]*(texture.shape[0]-1)).astype(int), 0, texture.shape[0]-1)
            pixels = texture[ty, tx]
            mask &= pixels[..., 3] > 128
            alpha = pixels[..., 3:4]/255.
            blended = pixels[..., :3]*alpha+self.rgb[region]*(1-alpha)
            self.rgb[region][mask] = blended[mask].astype(np.uint8)
        self.depth[region][mask] = z[mask]

    def model(self, model):
        for link in model.findall('link'):
            base = transform(model).dot(transform(link))
            for visual in link.findall('visual'):
                tf = base.dot(transform(visual))
                box = visual.find('geometry/box')
                mesh = visual.find('geometry/mesh')
                if box is not None:
                    size = np.array(list(map(float, box.findtext('size').split())))
                    color = np.array(list(map(float, visual.findtext('material/diffuse', '.9 .9 .9 1').split())))[:3]
                    for axis, sign in itertools.product(range(3), (-1, 1)):
                        normal = tf[:3, axis]*sign
                        if normal.dot(self.basis[2]) <= 0: continue
                        other = [i for i in range(3) if i != axis]
                        corners = np.zeros((4, 3)); corners[:, axis] = sign*size[axis]/2
                        corners[:, other] = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*size[other]/2
                        vertices = corners.dot(tf[:3, :3].T)+tf[:3, 3]
                        shade = .75+.25*max(0, normal.dot(np.array([-.3, .4, .866])))
                        for ids in ([0,1,2], [0,2,3]):
                            self.triangle(vertices[ids], color=np.uint8(np.clip(color*shade*255, 0, 255)))
                elif mesh is not None and mesh.findtext('uri', '').endswith('/sign.dae'):
                    if tf[:3, 0].dot(self.basis[2]) <= 0: continue
                    _, w, h = map(float, mesh.findtext('scale').split())
                    corners = np.array([[0,-w/2,h/2],[0,w/2,h/2],[0,w/2,-h/2],[0,-w/2,-h/2]])
                    vertices = corners.dot(tf[:3, :3].T)+tf[:3, 3]
                    name = visual.findtext('material/script/name').split('/')[-1]
                    pixels = np.asarray(Image.open(ROOT/'materials/textures'/(name+'.png')).convert('RGBA'))
                    uv = np.array([[0,0],[1,0],[1,1],[0,1]])
                    for ids in ([0,1,2], [0,2,3]):
                        self.triangle(vertices[ids], texture=pixels, uv=uv[ids])


def render(models, size, eye, target, span):
    renderer = Renderer(*size, eye, target, span)
    for model in models: renderer.model(model)
    return Image.fromarray(renderer.rgb)


def caption(image, title, subtitle):
    out = Image.new('RGB', (image.width, image.height+100), 'white')
    out.paste(image, (0, 100)); draw = ImageDraw.Draw(out)
    draw.text((28, 14), title, font=ImageFont.truetype(FONT, 28), fill='#182c48')
    draw.text((28, 58), subtitle, font=ImageFont.truetype(FONT, 18), fill='#506078')
    return out


def main():
    world = ET.parse(ROOT/'worlds/smart_community.world').getroot().find('world')
    models = world.findall('model')
    output = ROOT/'preview'; output.mkdir(exist_ok=True)
    overview = render(models, (1800, 1300), (-5, 6, 7), (0, 0, .05), 6.4)
    caption(overview, '9.24 规则场地 · 18 人 / 两个随机车牌 / 两处信号灯',
            '由实际 world 几何与原始贴图离线渲染；非 Gazebo 截图，未模拟光照、动态或识别。').save(output/'field_3d.png')
    cars = [m for m in models if m.get('name').startswith(('car_', 'license_plate_'))]
    car_view = render(cars, (1600, 470), (-4, -1.19, .5), (1.77, -1.19, .125), 1.8)
    caption(car_view, '车牌安装效果 · 左起 3 / 2 / 1 号车位',
            '1、2 号使用 doc 中两张原图；3 号使用官方示例牌。背景与车牌均为独立静态模型。').save(output/'plates.png')
    people = [m for m in models if m.get('name').startswith('person_')]
    # Inspection contact sheet: each original SDF model is viewed face-on,
    # without moving or changing it in the world.
    sheet = Image.new('RGB', (1800, 640), 'white'); draw = ImageDraw.Draw(sheet)
    for i, model in enumerate(people):
        tf = transform(model); center = tf[:3, 3]+[0,0,.075]
        tile = render([model], (200, 260), center+tf[:3, 0], center, .135)
        x, y = (i%9)*200, (i//9)*320
        sheet.paste(tile, (x,y))
        name = model.get('name')[7:]
        draw.text((x+8,y+270), name, font=ImageFont.truetype(FONT, 17), fill='#182c48')
    caption(sheet, '人物立牌正面检查 · A / B 区各 8 名社区人员 + 1 名非社区人员',
            '背板统一高 15 cm、宽 5 cm、厚 5 mm；原始人物图等比例居中。此图为逐模型正视图拼排。').save(output/'people.png')
    signals = [m for m in models if m.get('name') == 'north_signal']
    center = np.array([-.47,1.8,.24])
    signal = render(signals, (1100, 750), center+[2,0,0], center, .8)
    caption(signal, '信号灯正面 · 箱体 59 × 14 × 5 cm',
            '总宽 64 cm / 总高 48 cm；红 10 s → 绿 15 s → 黄 5 s。预览显示初始红灯。').save(output/'signal.png')
    print('Saved static SDF previews to '+str(output))


if __name__ == '__main__': main()
