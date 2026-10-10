"""Summarize real accepted frames per observation without changing recognition."""
from copy import deepcopy


class ObservationReports:
    def __init__(self):
        self.points={}

    def update(self,record,stable=None,targets=None):
        name=record['observation']
        if not name:return
        point=self.points.setdefault(name,dict(observation=name,task=record['task'],frames=0,
            people={},colors={},candidate=None))
        point['frames']+=1
        if record['task']=='person':
            for detection in record['detections']:
                person=point['people'].setdefault(detection['track_id'],dict(frames=0,scores={}))
                person['frames']+=1;role=detection['role']
                person['scores'][role]=max(person['scores'].get(role,0),detection['confidence'])
        elif record['task']=='signal' and stable not in (None,'unknown'):
            matching=[d['confidence'] for d in record['detections'] if d['color']==stable]
            if matching:
                color=point['colors'].setdefault(stable,dict(confidence=0.,frames=0))
                color['confidence']=max(color['confidence'],max(matching));color['frames']+=1
        elif record['task']=='plate' and targets:
            detection=targets[0]
            candidate=max(detection['ocr_candidates'],key=lambda c:c['ocr_confidence'],default=None)
            if candidate:
                value=dict(text=candidate['text'],ocr_confidence=candidate['ocr_confidence'],
                    detection_confidence=detection['confidence'])
                if point['candidate'] is None or value['ocr_confidence']>point['candidate']['ocr_confidence']:
                    point['candidate']=value

    def report(self,name,counts):
        point=self.points.get(name)
        if point is None:return dict(observation=name,task='unknown',frames=0)
        report={key:deepcopy(point[key]) for key in ('observation','task','frames')}
        if point['task']=='person':
            tracks={t['id']:t for c in counts.values() if isinstance(c,dict) for t in c.get('tracks',[])}
            report['people']=[]
            for track_id,evidence in point['people'].items():
                track=tracks.get(track_id)
                if track is None:continue  # Only confirmed tracks enter the accepted result.
                role=track['role'];report['people'].append(dict(track_id=track_id,role=role,
                    confidence=evidence['scores'].get(role,0),frames=evidence['frames']))
            report['pending']=len(point['people'])-len(report['people'])
        elif point['task']=='signal':report['colors']=deepcopy(point['colors'])
        elif point['task']=='plate':
            report['plate']=next((deepcopy(p) for p in counts.get('plates',[]) if p['observation']==name),None)
            report['candidate']=deepcopy(point['candidate'])
        return report

    def all(self,counts):
        return {name:self.report(name,counts) for name in self.points}
