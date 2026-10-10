"""Associate camera observations of standees; no scene population or identity lookup."""
import numpy as np


def project_person(box,intrinsics,camera_transform,height=.075):
    x1,y1,x2,y2=box
    u=(x1+x2)/2;v=(y1+y2)/2
    k=np.array(intrinsics).reshape(3,3);transform=np.array(camera_transform)
    ray=transform[:3,:3].dot(np.linalg.solve(k,[u,v,1.]))
    origin=transform[:3,3]
    if ray[2]>=-1e-5:return None
    distance=(height-origin[2])/ray[2]
    position=origin+distance*ray
    if distance<=0 or not .12<=np.linalg.norm(position[:2]-origin[:2])<=1.0:return None
    return position[:2].tolist()


class PersonTracker:
    def __init__(self,association_distance=.075,minimum_hits=3):
        self.radius=association_distance;self.minimum_hits=minimum_hits;self.tracks={'A':[],'B':[]}

    def update(self,district,observations):
        if district not in self.tracks:return []
        used=set();updated=[]
        for detection,position in sorted(observations,key=lambda o:o[0]['confidence'],reverse=True):
            location=np.array(position,dtype=float);candidates=[]
            for i,track in enumerate(self.tracks[district]):
                if i in used:continue
                distance=np.linalg.norm(location-np.array(track['position']))
                if distance<self.radius:candidates.append((distance,i))
            if candidates:index=min(candidates)[1]
            else:
                index=len(self.tracks[district]);self.tracks[district].append(dict(
                    id=district+'_person_'+str(index+1),district=district,position=location.tolist(),
                    hits=0,role_votes={'resident':0.,'visitor':0.},maximum_confidence=0.))
            used.add(index);track=self.tracks[district][index];track['hits']+=1
            track['position']=(.8*np.array(track['position'])+.2*location).tolist()
            track['role_votes'][detection['role']]+=detection['confidence']
            track['role']=max(track['role_votes'],key=track['role_votes'].get)
            track['maximum_confidence']=max(track['maximum_confidence'],detection['confidence'])
            track['confirmed']=track['hits']>=self.minimum_hits
            updated.append((dict(detection,track_id=track['id'],map_position=location.tolist(),
                confirmed=track['confirmed'],track_role=track['role']),track))
        return updated

    def counts(self):
        result={}
        for district,tracks in self.tracks.items():
            accepted=[t for t in tracks if t.get('confirmed')]
            result[district]=dict(total=len(accepted),residents=sum(t['role']=='resident' for t in accepted),
                visitors=sum(t['role']=='visitor' for t in accepted),tracks=accepted)
        return result
