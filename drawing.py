"""Square-run measurements and a top-down view of Gazebo body positions."""
import math
import time
from face import FACE

class Drawing:
    def __init__(self):
        self.clear()

    def clear(self):
        self.started_at = None
        self.finished_at = None
        self.mode = "square"
        self.origin = None
        self.points = []
        self.endpoint=(0.,0.)
        self.active = False
        self.completed = False
        self.distance = 0.
        self.last = None

    def begin(self, pose):
        self.started_at = time.monotonic()
        self.finished_at = None
        self.mode = "square"
        self.origin = pose
        self.points = [(0., 0.)]
        self.endpoint=(0.,0.)
        self.last = pose
        self.distance = 0.
        self.active = True
        self.completed = False

    def resume(self, pose):
        if self.origin is None:
            self.begin(pose)
        self.finished_at = None
        self.mode = "free"
        self.completed = False
        self.active = True
        self.last = pose

    def update(self, pose):
        if not self.active or not all(math.isfinite(v) for v in pose):
            return
        dx,dy = pose[0]-self.origin[0], pose[1]-self.origin[1]
        c,s = math.cos(self.origin[2]),math.sin(self.origin[2])
        point = (c*dx+s*dy, -s*dx+c*dy)
        self.distance += math.hypot(pose[0]-self.last[0],pose[1]-self.last[1])
        self.last = pose
        # Avoid repainting thousands of coincident samples while turning.
        if math.dist(point,self.points[-1])>=.001:
            self.points.append(point)
        self.endpoint=point

    def finish(self, completed=False):
        if self.active:
            self.finished_at = time.monotonic()
            self.completed = completed
            self.active = False

    @property
    def elapsed(self):
        if self.started_at is None:
            return 0.
        end=self.finished_at if self.finished_at is not None else time.monotonic()
        return max(0.,end-self.started_at)

    @property
    def gap_cm(self):
        return 100*math.hypot(*self.endpoint) if self.points else 0.

    @property
    def summary(self):
        if self.origin is None:
            return "前進・斜め移動で線を描けます。「四角を描いて」は新しい正方形を描きます。"
        state = "完了：出発点からのずれ" if self.completed else "描画中：出発点からの距離" if self.active else "中断：出発点からの距離"
        if self.mode == "free":
            state = "自由描画：出発点からの距離"
        timing = ""
        if self.mode == "face":
            result = " ／ 3分以内達成" if self.completed and self.elapsed<=180 else " ／ 3分超過" if self.elapsed>180 else " ／ 目標180秒"
            timing = f" ／ 時間 {self.elapsed:.1f}秒" + result
        return f"{state} {self.gap_cm:.1f} cm ／ 走行距離 {self.distance:.2f} m" + timing


def paint(canvas, drawing):
    canvas.delete("all")
    w,h = max(canvas.winfo_width(),400), max(canvas.winfo_height(),240)
    ideal=FACE if drawing.mode == "face" else [(0,0),(.5,0),(.5,.5),(0,.5),(0,0)]
    points=drawing.points
    all_points=ideal+points
    xmin=min(p[0] for p in all_points)-.2; xmax=max(p[0] for p in all_points)+.2
    ymin=min(p[1] for p in all_points)-.2; ymax=max(p[1] for p in all_points)+.2
    scale=min((w-50)/(xmax-xmin),(h-40)/(ymax-ymin))
    def xy(p):
        return (w/2+(p[0]-(xmin+xmax)/2)*scale,h/2-(p[1]-(ymin+ymax)/2)*scale)
    if len(points)>1:
        canvas.create_line(*[v for p in points for v in xy(p)],fill="#1678c8",width=3)
    x,y=xy((0,0))
    canvas.create_oval(x-5,y-5,x+5,y+5,fill="#23985a",outline="")
    canvas.create_text(x,y+17,text="出発点",fill="#17653d")
    if points:
        x,y=xy(points[-1])
        canvas.create_oval(x-5,y-5,x+5,y+5,fill="#ed8a22",outline="")
    canvas.create_text(12,12,anchor="nw",text="上から見た軌跡（ロボットの顔）" if drawing.mode == "face" else "上から見た軌跡（正方形：1辺0.5 m）" if drawing.mode == "square" else "上から見た軌跡（自由描画）",fill="#364656")
