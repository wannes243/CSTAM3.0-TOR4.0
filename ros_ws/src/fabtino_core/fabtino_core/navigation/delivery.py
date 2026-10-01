"""Delivery queue transitions independent of ROS timers and robot transport."""
import math


class Delivery:
    ACTIVE={'approaching','waiting','returning','planning'}
    def __init__(self):
        self.state='idle';self.tasks=[];self.index=0;self.deadline=None
        self.return_to_base=True;self.reason='';self.target=None;self.goal_id=None

    @property
    def active(self):return self.state in self.ACTIVE

    def start(self,tasks,zones,return_to_base=True):
        if not isinstance(tasks,list) or not 1<=len(tasks)<=128:raise ValueError('Add between 1 and 128 delivery tasks')
        zone_ids={zone['id'] for zone in zones};normalized=[]
        if not isinstance(return_to_base,bool):raise ValueError('Return to base must be true or false')
        for task in tasks:
            if not isinstance(task,dict):raise ValueError('Invalid delivery task')
            zone_id=str(task['zone_id']);delay=float(task.get('delay_s',0.))
            if zone_id not in zone_ids:raise ValueError('A delivery destination no longer exists')
            if not math.isfinite(delay) or not 0<=delay<=86400:raise ValueError('Task delay must be between 0 and 86400 seconds')
            normalized.append(dict(zone_id=zone_id,delay_s=delay,state='pending'))
        self.tasks=normalized;self.index=0;self.deadline=None;self.goal_id=None
        self.return_to_base=bool(return_to_base);self.reason='';self.state='planning';self.target=None

    def reached(self,now,goal_id):
        if goal_id!=self.goal_id:return
        if self.state=='returning':self.state='complete';self.goal_id=None;self.target=None
        elif self.state=='approaching':
            self.tasks[self.index]['state']='waiting';self.state='waiting'
            self.deadline=now+self.tasks[self.index]['delay_s'];self.goal_id=None

    def advance(self,now):
        if self.state!='waiting' or now<self.deadline:return False
        self.tasks[self.index]['state']='done';self.index+=1;self.deadline=None;self.target=None
        self.state='planning' if self.index<len(self.tasks) else ('returning' if self.return_to_base else 'complete')
        return True

    def cancel(self,reason='Cancelled by the operator'):
        if self.active:self.state='cancelled';self.reason=reason;self.goal_id=None;self.deadline=None;self.target=None

    def fail(self,reason):
        if self.index<len(self.tasks):self.tasks[self.index]['state']='failed'
        self.state='failed';self.reason=reason;self.goal_id=None;self.deadline=None;self.target=None

    def snapshot(self,now):
        return dict(state=self.state,tasks=[dict(task) for task in self.tasks],task_index=self.index,
                    completed=sum(task['state']=='done' for task in self.tasks),total=len(self.tasks),
                    remaining_s=max(0.,self.deadline-now) if self.deadline is not None else 0.,
                    return_to_base=self.return_to_base,reason=self.reason,target=self.target)
