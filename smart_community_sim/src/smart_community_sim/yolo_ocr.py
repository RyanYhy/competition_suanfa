"""Camera-only YOLO ONNX detection, lamp pixels and open-vocabulary plate OCR."""
import ast
import re
import time
from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort


def iou(a, boxes):
    low=np.maximum(a[:2],boxes[:,:2]);high=np.minimum(a[2:],boxes[:,2:])
    intersection=np.prod(np.maximum(0,high-low),axis=1)
    area=np.prod(a[2:]-a[:2]);areas=np.prod(boxes[:,2:]-boxes[:,:2],axis=1)
    return intersection/np.maximum(1e-8,area+areas-intersection)


class YoloOnnx:
    def __init__(self,path,confidence=.35,threads=2):
        options=ort.SessionOptions();options.intra_op_num_threads=threads;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
        self.input=self.session.get_inputs()[0];self.height,self.width=self.input.shape[2:]
        self.names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        self.confidence=confidence

    def detect(self,image):
        h,w=image.shape[:2];ratio=min(self.width/w,self.height/h)
        resized=cv2.resize(image,(round(w*ratio),round(h*ratio)))
        left=(self.width-resized.shape[1])//2;top=(self.height-resized.shape[0])//2
        canvas=np.full((self.height,self.width,3),114,np.uint8)
        canvas[top:top+resized.shape[0],left:left+resized.shape[1]]=resized
        tensor=np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
        output=self.session.run(None,{self.input.name:tensor})[0][0]
        if output.shape[0]==4+len(self.names):output=output.T
        if output.shape[1]!=4+len(self.names):raise ValueError('Unexpected YOLO output: '+str(output.shape))
        classes=output[:,4:].argmax(axis=1);scores=output[np.arange(len(output)),classes+4]
        keep=scores>=self.confidence;output=output[keep];classes=classes[keep];scores=scores[keep]
        boxes=np.column_stack((output[:,0]-output[:,2]/2,output[:,1]-output[:,3]/2,
                               output[:,0]+output[:,2]/2,output[:,1]+output[:,3]/2))
        boxes=(boxes-np.array([left,top,left,top]))/ratio
        boxes[:,[0,2]]=np.clip(boxes[:,[0,2]],0,w);boxes[:,[1,3]]=np.clip(boxes[:,[1,3]],0,h)
        result=[]
        for cls in sorted(set(classes)):
            candidates=np.flatnonzero(classes==cls);candidates=candidates[np.argsort(scores[candidates])[::-1]]
            while len(candidates):
                index=candidates[0];box=boxes[index]
                if min(box[2:]-box[:2])>=4:
                    result.append(dict(class_id=int(cls),class_name=self.names[int(cls)],
                        confidence=float(scores[index]),box=box.round().astype(int).tolist()))
                rest=candidates[1:];candidates=rest[iou(box,boxes[rest])<.45]
        return result


def lamp_color(image,box):
    x1,y1,x2,y2=box;crop=image[y1:y2,x1:x2]
    if not crop.size:return 'unknown',{}
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV);h,s,v=cv2.split(hsv)
    # Off lenses remain colored; select luminous pixels, not every lens pixel.
    saturated=(s>100)&(v>220)
    masks={'red':saturated&((h<12)|(h>168)), 'yellow':saturated&(h>=15)&(h<=38),
           'green':saturated&(h>=40)&(h<=95)}
    scores={name:int(mask.sum()) for name,mask in masks.items()}
    ranked=sorted(scores,key=scores.get,reverse=True);best,second=ranked[:2]
    if scores[best]<max(12,crop.shape[0]*crop.shape[1]*.006) or scores[best]<2.5*max(1,scores[second]):
        return 'unknown',scores
    return best,scores


def normalize_plate(text):
    # Separator/whitespace normalization only; never replace characters using known plates.
    return re.sub(r'[\s·•.\-]','',text).upper()


def valid_plate(text):
    return re.fullmatch(r'[京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼港澳][A-Z][A-Z0-9]{5,6}',text) is not None


class PlateOcr:
    def __init__(self):
        from rapidocr_onnxruntime import RapidOCR
        self.engine=RapidOCR(intra_op_num_threads=2,inter_op_num_threads=1,
                             use_cls=False,text_score=.4)

    def recognize(self,crop):
        if crop.size==0:return []
        # Text recognition directly on a plate crop; preserve Chinese characters.
        scale=max(1.,96/crop.shape[0]);large=cv2.resize(crop,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
        results=[]
        for name,image in [('original',large)]:
            raw,_=self.engine(image,use_det=False,use_cls=False,use_rec=True)
            for text,score in (raw or []):
                results.append(dict(text=normalize_plate(text),raw_text=text,ocr_confidence=float(score),preprocessing=name))
        return results


class YoloOcrRecognizer:
    def __init__(self,root,confidence=.35):
        self.root=Path(root)
        self.models={name:YoloOnnx(self.root/'model'/(file+'_best.onnx'),confidence) for name,file in
                     [('person','doll'),('plate','plate'),('signal','traffic_light')]}
        self.ocr=PlateOcr()

    def detect(self,image,task):
        start=time.monotonic()
        results=self.models[task].detect(image)
        if task=='person':
            selected=[]
            # Two classes may compete for one figure; count the visual object once.
            for d in sorted(results,key=lambda d:d['confidence'],reverse=True):
                if selected and np.max(iou(np.array(d['box']),np.array([p['box'] for p in selected])))>.45:continue
                selected.append(d)
            results=selected
        for d in results:
            d['category']=task
            if task=='person':d['role']='resident' if d['class_name']=='community' else 'visitor'
            if task=='signal':d['color'],d['pixel_scores']=lamp_color(image,d['box'])
            if task=='plate':
                x1,y1,x2,y2=d['box'];d['ocr_candidates']=self.ocr.recognize(image[y1:y2,x1:x2])
        return results,time.monotonic()-start
