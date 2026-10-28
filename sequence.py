"""Sequential square path using the existing single-movement controller."""
from motion import Motion
from drawing import Drawing
from face import FaceMotion

class SequenceMotion:
    def __init__(self):
        self.face = None
        self.motion = Motion()
        self.drawing = Drawing()
        self.steps = []
        self.sequence_name = "正方形"
        self.index = 0
        self.wait_since = None

    @property
    def pose(self):
        return self.motion.pose

    @property
    def message(self):
        if self.face is not None:
            return self.face.message
        prefix = f"{self.sequence_name} {self.index+1}/{len(self.steps)}：" if self.steps else ""
        return prefix + self.motion.message

    def update_pose(self, *args):
        self.motion.update_pose(*args)
        self.drawing.update(args[:3])
        if self.motion.message == "無効な位置情報":
            self.stop("無効な位置情報")

    def stop(self, message="停止"):
        self.drawing.finish(message.startswith("正方形走行完了") or message == "ロボットの顔 完了")
        self.face = None
        self.steps = []
        self.wait_since = None
        self.motion.stop(message)

    def clear_drawing(self):
        self.stop("絵を消しました")
        self.drawing.clear()

    def start(self, action, now):
        self.stop()
        if action == "face":
            self.motion.start("forward",now)
            if self.motion.action != "stop":
                self.motion.stop()
                self.face = FaceMotion(self.pose,now)
                self.drawing.begin(self.pose)
                self.drawing.mode = "face"
            return
        plans = {"square": (["forward", "left"] * 4, "正方形"),
                 "diagonal_right": (["right45", "forward"], "右斜め前進"),
                 "diagonal_left": (["left45", "forward"], "左斜め前進")}
        if action in plans:
            self.steps, self.sequence_name = plans[action]
            self.index = 0
            self.motion.start(self.steps[0], now)
        else:
            self.motion.start(action, now)
        if self.motion.action != "stop":
            if action == "square":
                self.drawing.begin(self.motion.pose)
            else:
                self.drawing.resume(self.motion.pose)

    def velocity(self, now):
        if self.face is not None:
            speed=self.face.velocity(self.pose,self.motion.received,now)
            if self.face.done:
                self.stop(self.face.message)
            return speed
        if not self.steps:
            speed = self.motion.velocity(now)
            if self.motion.action == "stop":
                self.drawing.finish()
            return speed
        if self.motion.received is None or now-self.motion.received > .6:
            self.stop("通信・シミュレーション停止のため中断")
            return 0., 0.
        if self.wait_since is not None:
            if now-self.wait_since < .2:
                return 0., 0.
            self.index += 1
            self.wait_since = None
            if self.index == len(self.steps):
                self.stop("正方形走行完了（1辺0.5 m × 4辺）" if self.sequence_name == "正方形" else self.sequence_name+"完了（45度旋回＋前進0.5 m）")
                return 0., 0.
            self.motion.start(self.steps[self.index], now)
        speed = self.motion.velocity(now)
        if self.motion.action == "stop":
            if "完了" in self.motion.message:
                self.wait_since = now
            else:
                self.stop(self.motion.message)
        return speed
