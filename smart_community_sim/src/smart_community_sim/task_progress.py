"""Human-readable task labels and concise console summaries."""
import json

LABELS={'observe_A_north':'A区北侧人物','observe_A_west':'A区西侧人物',
        'observe_A_south':'A区南侧人物','observe_B_north':'B区北侧人物',
        'observe_B_east':'B区东侧人物','observe_plate_1':'车牌1',
        'observe_plate_2':'车牌2','observe_plate_3':'车牌3',
        'north_wait':'北侧信号灯','south_wait':'南侧信号灯',
        'start_centerline':'驶入起点车道','north_crossing_exit':'通过北侧路口',
        'northwest':'西北转弯点','west_junction':'西侧路口',
        'middle_junction':'中间路口','south_crossing_exit':'通过南侧路口',
        'southeast':'东南转弯点','northeast':'东北路口','finish':'返回起点'}
COLORS={'red':'红灯','yellow':'黄灯','green':'绿灯'}


def observation_lines(report):
    name=report['observation'];label=LABELS.get(name,name)
    if not report.get('frames'):return ['[识别点结果] %s：未取得有效识别帧'%label]
    task=report['task'];lines=[]
    if task=='person':
        people=report.get('people',[]);residents=sum(p['role']=='resident' for p in people)
        lines=['[识别点结果] %s：确认%d人（社区%d，非社区%d），待确认%d；处理%d帧'%(
            label,len(people),residents,len(people)-residents,report.get('pending',0),report['frames'])]
        for person in people:
            short=person['track_id'].split('_')[0]+person['track_id'].split('_')[-1]
            lines.append('  %s %s：YOLO类别置信度（峰值）%.1f%%，观测%d帧'%(
                short,'社区人员' if person['role']=='resident' else '非社区人员',person['confidence']*100,person['frames']))
    elif task=='plate':
        plate=report.get('plate')
        if plate:
            lines=['[识别点结果] %s：%s；YOLO置信度 %.1f%%，OCR平均置信度 %.1f%%（%d帧确认）'%(
                label,plate['text'],plate['detection_confidence']*100,plate['ocr_confidence']*100,plate.get('confirmation_frames',3))]
        else:
            lines=['[识别点结果] %s：未确认车牌'%label];candidate=report.get('candidate')
            if candidate:lines.append('  候选 %s（未确认）：YOLO %.1f%%，OCR %.1f%%'%(
                candidate['text'],candidate['detection_confidence']*100,candidate['ocr_confidence']*100))
    elif task=='signal':
        colors=report.get('colors',{})
        seen=['%s（灯箱YOLO置信度 %.1f%%，%d帧）'%(COLORS[c],v['confidence']*100,v['frames']) for c,v in colors.items()]
        lines=['[识别点结果] %s：%s'%(label,'；'.join(seen) if seen else '未稳定识别灯色')]
    return lines or ['[识别点结果] %s：无有效识别结果'%label]


def mission_summary(points,reached,counts,reports,state,detail,require_vision=True):
    lines=['========== 任务汇总 ==========',
        '本轮状态：'+('全部完成' if state=='complete' else '未通过/已停止'),
        '导航航点：%d/%d'%(len(reached),len(points))]
    if require_vision:
        for district in ('A','B'):
            count=counts.get(district,{})
            lines.append('%s区人物：%d/9，社区%d，非社区%d'%(district,count.get('total',0),count.get('residents',0),count.get('visitors',0)))
        for point in points:
            name=point['name']
            if not ('district' in point or 'signal' in point or name.startswith('observe_plate')):continue
            report=reports.get(name,dict(observation=name,frames=0))
            lines.extend(line.replace('[识别点结果] ','',1) for line in observation_lines(report)[:1])
            if 'signal' in point:
                crossed=point['signal'].replace('_signal','_crossing_exit') in reached
                lines.append('  路口通行：'+('已到达路口出口' if crossed else '尚未通过'))
        lines.append('视觉验收：'+('通过' if counts.get('complete') else '未通过/未完成'))
    else:lines.append('视觉识别：未执行（导航诊断模式）')
    lines+=['结束说明：'+concise_detail(detail),'========== 汇总结束 ==========']
    return lines


def concise_detail(detail):
    """Keep machine JSON on ROS topics while printing only task totals."""
    prefix,separator,payload=detail.partition('：{')
    if not separator:return detail
    try:
        counts=json.loads('{'+payload)
        people=['%s区%d人（社区%d/非社区%d）'%(d,c['total'],c['residents'],c['visitors'])
                for d,c in counts.items() if isinstance(c,dict) and 'total' in c]
        plates=counts.get('plates',[])
        return prefix+'：'+'，'.join(people+['车牌%d个'%len(plates)])
    except (ValueError,KeyError,TypeError):return prefix
