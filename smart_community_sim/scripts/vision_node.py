#!/usr/bin/env python3
"""YOLO/OCR recognition from timestamped ROS camera frames. No simulator labels."""
import hashlib,json,threading,time
from collections import defaultdict
from pathlib import Path
import cv2,numpy as np
from PIL import ImageFont
import rospy,rospkg,tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse
from smart_community_sim.yolo_ocr import YoloOcrRecognizer,valid_plate
from smart_community_sim.observation_tracker import PersonTracker,project_person
from smart_community_sim.observation_report import ObservationReports
from smart_community_sim.run_results import RunResults
from smart_community_sim.recognition_overlay import annotate


class VisionNode:
    def __init__(self):
        cv2.setNumThreads(1)
        root=Path(rospkg.RosPack().get_path('smart_community_sim'))
        self.detector=YoloOcrRecognizer(root,float(rospy.get_param('~confidence',.15)))
        self.max_fps=float(rospy.get_param('~max_fps',5.))
        self.thresholds=rospy.get_param('~thresholds',{'person':.20,'plate':.35,'signal':.50})
        self.expected=rospy.get_param('~required_people',{'A':9,'B':9})
        self.ocr_threshold=float(rospy.get_param('~ocr_confidence',.65))
        self.output=Path(rospy.get_param('~output_dir',str(root/'validation')))
        self.bridge=CvBridge();self.listener=tf.TransformListener();self.lock=threading.RLock()
        self.pending=None;self.context={};self.generation=0;self.intrinsics=None;self.camera_info=None;self.reset_data()
        self.metadata=dict(backend='YOLO ONNX + PP-OCRv4',thresholds=self.thresholds,
            ocr_confidence=self.ocr_threshold,required_people=self.expected,
            models={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'model').glob('*.onnx')})
        self.store=RunResults(self.output,self.expected,rospy.get_param('/patrol/waypoints',[]),
            save_debug=rospy.get_param('~save_debug',False),
            debug_root=rospy.get_param('~debug_output_dir',str(root.parent/'doc/validation/识别调试结果')))
        self.store.begin(self.metadata)
        self.font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',20)
        self.image_pub=rospy.Publisher('/vision/annotated',Image,queue_size=1)
        self.results=rospy.Publisher('/vision/detections',String,queue_size=20)
        self.summary=rospy.Publisher('/vision/district_counts',String,queue_size=1,latch=True)
        self.signals={n:rospy.Publisher('/vision/traffic_lights/'+n+'/state',String,queue_size=1)
                      for n in ('north_signal','south_signal')}
        self.subscribers=[rospy.Subscriber('/front_camera/image_raw',Image,self.camera,queue_size=1,buff_size=2**24),
            rospy.Subscriber('/front_camera/camera_info',CameraInfo,self.info,queue_size=1),
            rospy.Subscriber('/patrol/observation_context',String,self.observe,queue_size=10),
            rospy.Subscriber('/patrol/status',String,self.patrol_status,queue_size=10),
            rospy.Subscriber('/patrol/goal_reached',String,self.goal_reached,queue_size=20)]
        rospy.Service('/vision/reset',Trigger,self.reset);rospy.Service('/vision/finalize',Trigger,self.finalize)
        rospy.Service('/vision/observation_report',Trigger,self.observation_report)
        rospy.on_shutdown(self.shutdown)
        threading.Thread(target=self.work,daemon=True).start()
        ROS_INFO={'backend':'YOLO ONNX + PP-OCRv4','thresholds':self.thresholds,'models':str(root/'model')}
        rospy.loginfo('视觉配置 %s',json.dumps(ROS_INFO,ensure_ascii=False))

    def reset_data(self):
        self.tracker=PersonTracker();self.plate_votes=defaultdict(lambda:defaultdict(list));self.plates={}
        self.observations=ObservationReports()
        self.sequence=0;self.events=set();self.signal_candidate=None;self.signal_frames=0
        self.session=str(time.time_ns());self.last_signal={};self.last_saved={};self.last_stamp=None

    def reset(self,_):
        with self.lock:
            self.reset_data();self.pending=None;self.generation+=1
            self.store.begin(self.metadata,self.camera_info)
            self.summary.publish(json.dumps(self.counts(),ensure_ascii=False))
        return TriggerResponse(True,'YOLO/OCR统计已重置，上一轮生成结果已覆盖')

    def info(self,msg):
        with self.lock:
            self.intrinsics=list(msg.K)
            self.camera_info=dict(K=list(msg.K),D=list(msg.D),width=msg.width,height=msg.height,frame=msg.header.frame_id)
            path=self.store.debug_root/'records/camera_info.json'
            if self.store.save_debug and not path.exists():self.store.write_json(path,self.camera_info)

    def patrol_status(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:self.store.status(data['state'],data['detail'])

    def goal_reached(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:self.store.goal(data['name'])

    def camera(self,msg):
        with self.lock:
            if self.last_stamp==msg.header.stamp:return
            self.pending=(msg,dict(self.context),self.generation);self.last_stamp=msg.header.stamp

    def observe(self,msg):
        try:context=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:
            self.context=context;self.generation+=1;self.signal_candidate=None;self.signal_frames=0

    def counts(self):
        result=self.tracker.counts();result['plates']=list(self.plates.values())
        result['observations']=self.observations.all(result);return result

    def observation_report(self,_):
        with self.lock:
            name=self.context.get('name','');report=self.observations.report(name,self.counts())
        return TriggerResponse(bool(report['frames']),json.dumps(report,ensure_ascii=False))

    def shutdown(self):
        with self.lock:
            if self.store.state not in ('complete','stopped'):
                self.store.status('stopped','节点已退出，保留已取得的识别图片')

    def finalize(self,_):
        with self.lock:
            counts=self.counts()
            roles_ok=sum(counts[d]['residents'] for d in self.expected)==16 and sum(counts[d]['visitors'] for d in self.expected)==2
            complete=all(counts[d]['total']==self.expected[d] for d in self.expected) and roles_ok and len({p['text'] for p in self.plates.values()})==3
            counts['complete']=complete;counts['backend']='YOLO ONNX + PP-OCRv4';text=json.dumps(counts,ensure_ascii=False)
            self.store.finalize(counts);self.summary.publish(text)
            rospy.logdebug('视觉最终统计 %s',text)
        return TriggerResponse(complete,text)

    def work(self):
        next_frame=0.
        while not rospy.is_shutdown():
            if time.monotonic()<next_frame:time.sleep(.02);continue
            with self.lock:item=self.pending;self.pending=None
            if item is None:time.sleep(.02);continue
            msg,context,generation=item
            if not 0<=rospy.Time.now().to_sec()-msg.header.stamp.to_sec()<=.8:continue
            task='signal' if context.get('signal') else 'person' if context.get('district') else 'plate' if context.get('name','').startswith('observe_plate') else None
            if task is None:continue
            try:
                frame=self.bridge.imgmsg_to_cv2(msg,'bgr8');detections,elapsed=self.detector.detect(frame,task)
                detections=[d for d in detections if d['confidence']>=self.thresholds[task]]
                transform=None
                if task=='person' and self.intrinsics:
                    pos,quat=self.listener.lookupTransform('map',msg.header.frame_id,msg.header.stamp)
                    transform=tf.transformations.quaternion_matrix(quat);transform[:3,3]=pos
                next_frame=time.monotonic()+1/self.max_fps
                with self.lock:
                    if generation!=self.generation or not 0<=rospy.Time.now().to_sec()-msg.header.stamp.to_sec()<=1.:continue
                    self.publish(msg,frame,detections,context,task,elapsed,transform)
            except (tf.Exception,ValueError,cv2.error) as exc:rospy.logwarn_throttle(2,'视觉帧处理：%s',str(exc))

    def publish(self,msg,frame,detections,context,task,elapsed,transform):
        events=[];name=context.get('name','')
        if task=='signal':
            colors={d['color'] for d in detections if d['color']!='unknown'}
            state=next(iter(colors)) if len(colors)==1 else 'unknown'
            self.signal_frames=self.signal_frames+1 if state==self.signal_candidate else 1;self.signal_candidate=state
            stable=state if self.signal_frames>=2 else 'unknown'
            self.signals[context['signal']].publish(stable)
            self.store.signal(context['signal'],stable)
            if stable!='unknown' and stable!=self.last_signal.get(context['signal']):
                self.last_signal[context['signal']]=stable;events.append(dict(category='signal',signal=context['signal'],label=stable,detections=detections))
        elif task=='person' and transform is not None:
            observations=[]
            for d in detections:
                if d['box'][3]-d['box'][1]<frame.shape[0]*.08:continue
                position=project_person(d['box'],self.intrinsics,transform)
                if position is not None:observations.append((d,position))
            updates=self.tracker.update(context['district'],observations)
            detections=[d for d,t in updates]
            for d,t in updates:
                if t['confirmed'] and t['id'] not in self.events:
                    self.events.add(t['id']);events.append(dict(d,label='社区人员' if t['role']=='resident' else '非社区人员',district=context['district']))
        elif task=='plate':
            # A wide view can contain neighboring cars. Observe the plate at the
            # camera center; a neighboring plate must not fill this parking slot.
            targets=[d for d in detections if abs((d['box'][0]+d['box'][2])/2-frame.shape[1]/2)<frame.shape[1]*.2]
            if targets:targets=[min(targets,key=lambda d:abs((d['box'][0]+d['box'][2])/2-frame.shape[1]/2))]
            for d in targets:
                for c in d['ocr_candidates']:
                    if c['ocr_confidence']>=self.ocr_threshold and valid_plate(c['text']):
                        self.plate_votes[name][c['text']].append(c['ocr_confidence'])
                        votes=self.plate_votes[name][c['text']]
                        if len(votes)>=3 and name not in self.plates:
                            plate=dict(observation=name,text=c['text'],ocr_confidence=sum(votes)/len(votes),
                                detection_confidence=d['confidence'],confirmation_frames=len(votes))
                            self.plates[name]=plate;events.append(dict(d,**plate,label=c['text']))
        annotated=annotate(frame,detections,task,self.font)
        out=self.bridge.cv2_to_imgmsg(annotated,'bgr8');out.header=msg.header;self.image_pub.publish(out)
        record=dict(stamp=msg.header.stamp.to_sec(),frame=int(msg.header.seq),observation=name,task=task,inference_s=elapsed,
            processing_age_s=rospy.Time.now().to_sec()-msg.header.stamp.to_sec(),detections=detections)
        self.observations.update(record,stable=stable if task=='signal' else None,targets=targets if task=='plate' else None)
        self.store.frame(record)
        if task=='person' and detections:
            confirmed=[d for d in detections if d['confirmed']]
            score=(len(confirmed),len(confirmed)-len(detections),sum(d['confidence'] for d in confirmed))
            self.store.save_image(name,frame,annotated,record,score)
        elif task=='plate' and targets:
            d=targets[0];ocr=max(d['ocr_candidates'],key=lambda c:c['ocr_confidence'],default={})
            accepted=self.plates.get(name,{}).get('text')
            score=(accepted is not None and ocr.get('text')==accepted,ocr.get('ocr_confidence',0),d['confidence'])
            x,y,x2,y2=d['box'];self.store.save_image(name,frame,annotated,record,score,crop=frame[y:y2,x:x2])
        elif task=='signal' and stable!='unknown':
            known=[d for d in detections if d['color']!='unknown']
            score=(len(known)-len(detections),max((d['confidence'] for d in known),default=0))
            self.store.save_image(name+'_'+stable,frame,annotated,record,score)
        for event in events:
            self.sequence+=1;event.update(sequence=self.sequence,stamp=msg.header.stamp.to_sec(),frame=int(msg.header.seq),observation=name)
            self.store.event(event,annotated)
            encoded=json.dumps(event,ensure_ascii=False);rospy.logdebug('识别结果 %s',encoded);self.results.publish(encoded)
        counts=self.counts();self.store.update_counts(counts,force=bool(events));self.summary.publish(json.dumps(counts,ensure_ascii=False))

if __name__=='__main__':
    rospy.init_node('community_vision');VisionNode();rospy.spin()
