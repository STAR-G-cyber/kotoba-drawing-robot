"""Continuous robot face path, tracked using Gazebo body pose."""
import math

# Same visible face, ordered to avoid redundant border traversals.
FACE = [(0,0),(0,.15),(.45,.15),(0,.15),(0,.425),(.125,.425),(.125,.325),
        (.225,.325),(.225,.425),(.375,.425),(.375,.325),(.475,.325),(.475,.425),
        (0,.425),(0,.6),(.25,.6),(.25,.75),(.35,.75),(.35,.6),(.6,.6),(.6,0),(0,0)]

def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))

class FaceMotion:
    def __init__(self, pose, now):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        self.targets=[(x+c*u-s*v,y+s*u+c*v) for u,v in FACE[1:]]
        self.index=0
        self.segment_started=now
        self.arrived=None
        self.done=False
        self.success=False
        self.message="顔を描いています"
        self.previous_pose=pose
        self.previous_received=now
        self.speed=0.
        self.turn_rate=0.
        self.phase="align"
        self.start_point=(x,y)
        self.direction=1
        self._prepare(pose)

    def _prepare(self, pose):
        x,y=self.targets[self.index]
        dx,dy=x-self.start_point[0],y-self.start_point[1]
        self.length=math.hypot(dx,dy)
        self.heading=math.atan2(dy,dx)
        # Retrace with reverse motion when it avoids a 180-degree turn.
        self.direction=1 if abs(wrap(self.heading-pose[2]))<=math.pi/2 else -1
        self.body_heading=wrap(self.heading if self.direction==1 else self.heading+math.pi)
        self.phase="align"
        self.arrived=None

    def velocity(self, pose, received, now):
        if self.done:
            return 0.,0.
        if received is None or now-received>.6:
            self.done=True;self.message="通信・シミュレーション停止のため中断"
            return 0.,0.
        if now-self.segment_started>40:
            self.done=True;self.message="時間上限で中断"
            return 0.,0.
        if received>self.previous_received:
            dt=received-self.previous_received
            self.speed=math.hypot(pose[0]-self.previous_pose[0],pose[1]-self.previous_pose[1])/dt
            self.turn_rate=abs(wrap(pose[2]-self.previous_pose[2]))/dt
            self.previous_pose=pose
            self.previous_received=received
        x,y=self.targets[self.index]
        dx,dy=x-pose[0],y-pose[1]
        distance=math.hypot(dx,dy)
        self.message=f"ロボットの顔 {self.index+1}/{len(self.targets)}：描画中"
        if distance<.01:
            if self.speed<.012 and self.turn_rate<.025:
                if self.arrived is None:
                    self.arrived=received
                elif received-self.arrived>=.18:
                    self.start_point=self.targets[self.index]
                    self.index+=1;self.arrived=None;self.segment_started=now
                    if self.index==len(self.targets):
                        self.done=True;self.success=True;self.message="ロボットの顔 完了"
                    else:
                        self._prepare(pose)
            else:
                self.arrived=None
            return 0.,0.
        self.arrived=None
        heading_error=wrap(self.body_heading-pose[2])
        if self.phase=="align":
            if abs(heading_error)<.025 and self.speed<.012 and self.turn_rate<.04:
                self.phase="drive"
            return 0.,max(-1.4,min(1.4,2.2*heading_error))
        c,s=math.cos(self.heading),math.sin(self.heading)
        along=dx*c+dy*s
        cross=-(pose[0]-self.start_point[0])*s+(pose[1]-self.start_point[1])*c
        correction=max(-.15,min(.15,math.atan2(-cross,.12)))
        desired=wrap(self.body_heading+correction*(1 if along>=0 else -1))
        error=wrap(desired-pose[2])
        angular=max(-.4,min(.4,1.8*error))
        # No minimum speed: short eye strokes and corners must brake to zero.
        limit=.15 if self.length<=.16 else .8
        travel=max(-limit,min(limit,2.5*along))
        if abs(along)<.008 and distance>=.01:
            bearing=math.atan2(dy,dx)
            reverse=abs(wrap(bearing-pose[2]))>math.pi/2
            error=wrap(bearing+(math.pi if reverse else 0)-pose[2])
            angular=max(-.4,min(.4,1.8*error))
            travel=(-1 if reverse else 1)*min(.04,distance)
            linear=travel if abs(error)<.08 else 0.
        else:
            linear=self.direction*travel if abs(error)<.12 else 0.
        return linear,angular
