"""Pure feedback controller, independent of ROS; simulator use only."""
import math
class Motion:
    def __init__(self):
        self.pose = None
        self.received = None
        self.action = "stop"
        self.origin = None
        self.started = 0
        self.angle = 0
        self.settled_since = None
        self.message = "Gazeboの位置情報を待っています"
    def update_pose(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x,y,yaw,now)):
            self.stop("無効な位置情報")
            return
        if self.pose and self.action in ("left", "right", "spin", "left45", "right45"):
            delta = yaw-self.pose[2]
            self.angle += math.atan2(math.sin(delta), math.cos(delta))
        self.pose = (x,y,yaw)
        self.received = now
    def stop(self, message="停止"):
        self.action = "stop"
        self.settled_since = None
        self.message = message
    def start(self, action, now):
        self.stop()
        if action == "stop":
            return
        if action not in ("forward","backward","left","right","spin","left45","right45"):
            return
        if self.received is None or now-self.received > .6:
            self.message = "Gazeboの位置情報が届いていません"
            return
        self.action, self.origin, self.started = action, self.pose, now
        self.angle = 0
        self.settled_since = None
        self.message = "動作中"
    def velocity(self, now):
        if self.action == "stop":
            return 0., 0.
        if self.received is None or now-self.received > .6:
            self.stop("通信・シミュレーション停止のため中断")
        elif now-self.started > (60 if self.action == "spin" else 25 if self.action in ("left", "right", "left45", "right45") else 12):
            self.stop("時間上限で中断")
        elif self.action in ("forward", "backward"):
            remaining = .5-math.hypot(self.pose[0]-self.origin[0], self.pose[1]-self.origin[1])
            if remaining <= .02:
                self.stop("後退完了（目標0.5 m）" if self.action == "backward" else "前進完了（目標0.5 m）")
            else:
                return (-1 if self.action == "backward" else 1) * min(1., max(.05, remaining*1.6)), 0.
        else:
            sign = -1 if self.action in ("right", "right45") else 1
            target_degrees = 360 if self.action == "spin" else sign*(45 if self.action in ("left45", "right45") else 90)
            error = math.radians(target_degrees)-self.angle
            self.message = f"旋回中：移動角 {math.degrees(self.angle):+.1f}度 / 目標 {target_degrees:+d}度"
            # Stop sending rotation within tolerance, but keep observing fresh
            # odometry for coast / overshoot before declaring completion.
            if abs(error) <= math.radians(0.7):
                if self.settled_since is None:
                    self.settled_since = self.received
                elif self.received-self.settled_since >= 0.3:
                    self.stop(f"旋回完了：移動角 {math.degrees(self.angle):+.1f}度（目標 {target_degrees:+d}度）")
                return 0., 0.
            self.settled_since = None
            # Signed feedback also corrects a small overshoot. No minimum
            # angular speed: approach the target gently instead of coasting in.
            return 0., max(-0.8, min(0.8, error*2.8))
        return 0., 0.

