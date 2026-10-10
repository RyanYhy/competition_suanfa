"""Readable annotations on actual camera frames and their detection records."""
import cv2
import numpy as np
from PIL import Image,ImageDraw


def annotate(frame,detections,task,font):
    canvas=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(canvas)
    for d in detections:
        color='#00ff00';label=d['class_name']+' %.2f'%d['confidence']
        if task=='person':
            track=d['track_id'];short=track.split('_')[0]+track.split('_')[-1]
            role=d.get('track_role',d['role'])
            label=short+' %.2f'%d['confidence']+'\n'+('社区人员' if role=='resident' else '非社区人员')
            if not d.get('confirmed'):label+='\n待确认';color='#ffff00'
            elif role=='visitor':color='#ff8040'
        elif task=='plate' and d.get('ocr_candidates'):label+=' '+d['ocr_candidates'][0]['text']
        elif task=='signal':label+=' '+d['color']
        lines=label.splitlines();width=max(font.getsize(line)[0] for line in lines)
        x=max(0,min(d['box'][0],frame.shape[1]-width));y=max(0,d['box'][1]-24*len(lines))
        draw.rectangle(d['box'],outline=color,width=2)
        draw.multiline_text((x,y),label,font=font,fill=color,spacing=2)
    return cv2.cvtColor(np.array(canvas),cv2.COLOR_RGB2BGR)
