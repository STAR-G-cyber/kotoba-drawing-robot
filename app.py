"""ROS 2 / Gazebo Japanese command UI. Run with Ubuntu system Python."""
import math
import time
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from intent import Classifier, LABELS
from drawing import paint
from sequence import SequenceMotion as Motion

class Control(Node):
    def __init__(self):
        super().__init__("kotoba_control")
        self.motion = Motion()
        self.sim_start = None
        self.wall_start = None
        self.sim_ratio = None
        self.pub = self.create_publisher(Twist, "/kotoba/cmd_vel", 10)
        self.create_subscription(Odometry, "/kotoba/odom", self.odom, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)
    def odom(self, msg):
        now=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.sim_start is None or stamp < self.sim_start:
            self.sim_start,self.wall_start=stamp,now
        elif now-self.wall_start>=2:
            self.sim_ratio=(stamp-self.sim_start)/(now-self.wall_start)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.motion.update_pose(p.x,p.y,yaw,time.monotonic())
    def tick(self):
        v,w = self.motion.velocity(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.angular.z = v,w
        self.pub.publish(msg)

def main():
    rclpy.init()
    node, ai = Control(), Classifier()
    root = tk.Tk()
    root.title("ことばで描くロボット | ROS 2 + Gazebo")
    root.geometry("900x760")
    ttk.Label(root,text="ことばで描くロボット",font=("",20)).pack(pady=16)
    ttk.Label(root,text="前後0.5 m・斜め前進（45度旋回＋0.5 m）・左右90度・360度・正方形・顔（顔：3分以内を目標）").pack()
    ttk.Label(root,text="右の▼から例文を選べます。自由な文章はWindowsで入力し、ここへ貼り付けてください。").pack(pady=6)
    entry = ttk.Combobox(root,font=("",16),width=42,state="normal",
                         values=("ロボットの顔を描いて", "右斜めに進んで", "左斜めに進んで", "四角を描いて", "正方形に走って", "四角く一周して", "360度回転", "前に進んで", "バックして", "後ろに進んで", "右を向いて", "左を向いて", "止まって",
                                 "まっすぐ進んでください", "右に曲がってください",
                                 "左に回ってください", "今すぐ止まって", "明日の天気を教えて"))
    entry.pack(pady=12)
    status, pose = tk.StringVar(), tk.StringVar()
    history = tk.Text(root,height=4,state="disabled")
    def record(text):
        history.configure(state="normal")
        history.insert("end",text+"\n")
        history.see("end")
        history.configure(state="disabled")
    def command(event=None):
        text = entry.get()
        label,score,reason = ai.predict(text)
        node.motion.start(label or "stop",time.monotonic())
        node.tick()
        record(f"入力：{text} → "+(f"{LABELS[label]} / スコア {score:.2f}" if label else reason))
    def stop():
        node.motion.stop("停止ボタン")
        node.tick()
    def clear_drawing():
        node.motion.clear_drawing()
        node.tick()
        record("絵と計測結果を消しました")
    buttons = ttk.Frame(root)
    buttons.pack()
    ttk.Button(buttons,text="AIで判断して実行",command=command).pack(side="left",padx=8)
    ttk.Button(buttons,text="停止",command=stop).pack(side="left",padx=8)
    ttk.Button(buttons,text="絵を消す",command=clear_drawing).pack(side="left",padx=8)
    ttk.Label(root,textvariable=status).pack(pady=6)
    ttk.Label(root,textvariable=pose).pack()
    measurement = tk.StringVar()
    ttk.Label(root,textvariable=measurement,font=("",11),wraplength=850).pack(pady=6)
    canvas = tk.Canvas(root,height=320,background="#f5f8fc",highlightthickness=0)
    canvas.pack(fill="both",expand=True,padx=20,pady=6)
    ttk.Label(root,text="青：Gazeboの車体位置から記録 ／ 緑：出発点 ／ 橙：現在・終了位置").pack()
    history.pack(fill="x",padx=20,pady=8)
    def refresh_drawing():
        measurement.set(node.motion.drawing.summary)
        paint(canvas,node.motion.drawing)
        root.after(200,refresh_drawing)
    refresh_drawing()
    entry.bind("<Return>",command)
    root.bind("<Escape>",lambda e:stop())
    def pump():
        if not rclpy.ok():
            root.destroy()
            return
        rclpy.spin_once(node,timeout_sec=0)
        status.set(node.motion.message)
        p = node.motion.pose
        progress="" if node.sim_ratio is None else f" ／ シミュレーション進行率 約{node.sim_ratio*100:.0f}%"
        pose.set("位置情報：待機中" if p is None else f"x={p[0]:.2f} m  y={p[1]:.2f} m  向き={math.degrees(p[2]):.1f}°"+progress)
        root.after(10,pump)
    def close():
        node.motion.stop("終了")
        for _ in range(3):
            node.tick()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW",close)
    try:
        pump()
        root.mainloop()
    finally:
        if rclpy.ok():
            node.motion.stop()
            node.tick()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()

