"""Latest run: annotated images, overview and terminal log; debug evidence elsewhere."""
import json,re,shutil,time
from pathlib import Path
import cv2
from PIL import Image,ImageDraw,ImageFont
from .task_progress import LABELS,concise_detail
from .terminal_log import LOG_NAME


class RunResults:
    DIRECTORIES=('yolo_images','raw_images','plate_crops','records','navigation')
    FILES=('任务完成情况.md','任务完成情况.json','识别总览.jpg',LOG_NAME,
           'vision_evaluation.json','evaluation.log','recognition_preview.jpg','recognition_preview_sources.json')

    def __init__(self,root,expected,route,save_debug=False,debug_root=None):
        self.root=Path(root).expanduser()
        self.debug_root=Path(debug_root).expanduser() if debug_root else self.root.with_name(self.root.name+'_debug')
        package_root=Path(__file__).resolve().parents[2]
        forbidden={Path('/'),Path.home(),package_root,*package_root.parents}
        for directory in (self.root,self.debug_root):
            if directory.is_symlink() or directory.resolve() in forbidden:
                raise ValueError('Result directory must be a dedicated folder')
        if self.root.resolve()==self.debug_root.resolve() or self.root.resolve() in self.debug_root.resolve().parents or self.debug_root.resolve() in self.root.resolve().parents:
            raise ValueError('Debug and public results must use separate directories')
        self.expected=expected;self.route=route;self.save_debug=save_debug

    @staticmethod
    def marker(root):return root.parent/('.'+root.name+'.community_results')

    def reset_folder(self,root,directories,files):
        root.mkdir(parents=True,exist_ok=True)
        marker=self.marker(root);legacy=root/'.community_results'
        if not marker.exists() and not legacy.exists():
            for path in root.iterdir():
                if path.is_symlink() or path.is_file() or any(path.iterdir()):
                    raise ValueError('Refusing to clear unowned generated directory: '+str(root))
        marker.write_text('smart_community_sim generated latest patrol results\n')
        for name in directories:
            path=root/name
            if path.is_symlink():raise ValueError('Generated directory is a symlink: '+str(path))
            if path.exists():shutil.rmtree(str(path))
        for name in files:
            path=root/name
            if path.exists():path.unlink()
        if legacy.exists():legacy.unlink()

    def write_json(self,path,data):
        path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)

    def begin(self,metadata,camera_info=None):
        self.reset_folder(self.root,self.DIRECTORIES,self.FILES)
        (self.root/'yolo_images').mkdir();(self.root/LOG_NAME).touch()
        self.run_id=str(time.time_ns());self.state='ready';self.detail='等待开始'
        self.goals=[];self.counts={d:dict(total=0,residents=0,visitors=0,tracks=[]) for d in self.expected}
        self.counts['plates']=[];self.lamps={name:dict(colors=[],crossed=False) for name in ('north_signal','south_signal')}
        self.images={};self.image_scores={};self.last_write=0
        if self.save_debug:
            self.reset_folder(self.debug_root,('raw_images','plate_crops','records'),self.FILES)
            for name in ('raw_images','plate_crops','records/events'):(self.debug_root/name).mkdir(parents=True)
            self.write_json(self.debug_root/'records/configuration.json',dict(metadata,run_id=self.run_id))
            if camera_info:self.write_json(self.debug_root/'records/camera_info.json',camera_info)
        self.overview();self.flush(force=True)

    def status(self,state,detail):
        self.state=state;self.detail=LABELS.get(detail,concise_detail(detail))
        self.flush(force=True)
        if state in ('complete','stopped'):self.overview()

    def goal(self,name):
        if name not in self.goals:self.goals.append(name)
        for lamp in self.lamps:
            if name==lamp.replace('_signal','_crossing_exit'):self.lamps[lamp]['crossed']=True
        self.flush(force=True)

    def signal(self,name,color):
        if color!='unknown' and color not in self.lamps[name]['colors']:
            self.lamps[name]['colors'].append(color);self.flush(force=True)

    def update_counts(self,counts,force=False):
        self.counts=counts;self.flush(force=force)

    def save_image(self,name,frame,annotated,record,score,crop=None):
        key=re.sub(r'[^A-Za-z0-9_-]','_',name);score=tuple(score)
        if key in self.image_scores and score<=self.image_scores[key]:return False
        self.image_scores[key]=score;path=self.root/'yolo_images'/(key+'.png')
        cv2.imwrite(str(path),annotated)
        self.images[key]=dict(image=str(path.relative_to(self.root)),stamp=record['stamp'],
            frame=record['frame'],observation=record['observation'])
        if self.save_debug:
            raw=self.debug_root/'raw_images'/(key+'.png');cv2.imwrite(str(raw),frame)
            if crop is not None:cv2.imwrite(str(self.debug_root/'plate_crops'/(key+'.png')),crop)
            self.images[key].update(raw=str(raw.relative_to(self.debug_root)),detections=record['detections'])
            self.write_json(self.debug_root/'records/images.json',self.images)
        return True

    def frame(self,record):
        if not self.save_debug:return
        with (self.debug_root/'records/frames.jsonl').open('a') as stream:stream.write(json.dumps(record,ensure_ascii=False)+'\n')

    def event(self,event,annotated):
        if not self.save_debug:return
        path=self.debug_root/'records/events'/('%04d.png'%event['sequence'])
        cv2.imwrite(str(path),annotated);event['image']=str(path.relative_to(self.debug_root))
        with (self.debug_root/'records/events.jsonl').open('a') as stream:stream.write(json.dumps(event,ensure_ascii=False)+'\n')

    def finalize(self,summary):
        if self.save_debug:self.write_json(self.debug_root/'records/summary.json',summary)
        self.update_counts(summary,force=True)

    def flush(self,force=False):
        if not self.save_debug:return
        if not force and time.monotonic()-self.last_write<.5:return
        self.last_write=time.monotonic()
        self.write_json(self.debug_root/'records/status.json',dict(run_id=self.run_id,state=self.state,
            detail=self.detail,goals=self.goals,counts=self.counts,signals=self.lamps,images=self.images))

    def overview(self):
        selected=[]
        for point in self.route:
            name=point['name'];key=name+'_green' if 'signal' in point else name
            if key not in self.images and 'signal' in point:
                key=next((k for k,v in self.images.items() if v['observation']==name),key)
            if key in self.images:selected.append((point,self.images[key]))
        font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',22)
        canvas=Image.new('RGB',(1280,60+536*max(1,(len(selected)+1)//2)),(18,22,28));draw=ImageDraw.Draw(canvas)
        state={'ready':'等待开始','navigating':'巡航中','waiting_signal':'等待绿灯','complete':'全部完成','stopped':'未通过/已停止'}.get(self.state,self.state)
        draw.text((12,12),'巡航识别总览　状态：'+state,font=font,fill=(235,240,250))
        if not selected:draw.text((24,100),'尚无已保存的识别图片',font=font,fill=(235,240,250))
        for index,(point,record) in enumerate(selected):
            x=index%2*640;y=60+index//2*536
            canvas.paste(Image.open(self.root/record['image']).convert('RGB').resize((640,480)),(x,y+56))
            label=LABELS.get(point['name'],point['name'])
            plate=next((p for p in self.counts.get('plates',[]) if p['observation']==point['name']),None)
            if plate:label+=' '+plate['text']
            draw.text((x+8,y+10),label+' t=%.2fs'%record['stamp'],font=font,fill=(235,240,250))
        canvas.save(self.root/'识别总览.jpg',quality=94)
