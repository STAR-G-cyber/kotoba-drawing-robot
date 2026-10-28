import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from motion import Motion
from intent import Classifier

class Tests(unittest.TestCase):
    def test_face_at_slow_simulation_rate(self):
        from face import FaceMotion
        f=FaceMotion((0,0,0),0);x=y=yaw=v=w=now=0.;dt=.02;factor=.37
        for _ in range(12000):
            now+=dt;a,b=f.velocity((x,y,yaw),now,now)
            v+=(a-v)*dt*factor/.15;w+=(b-w)*dt*factor/.15
            x+=v*math.cos(yaw)*dt*factor;y+=v*math.sin(yaw)*dt*factor;yaw+=w*dt*factor
            if f.done:break
        self.assertTrue(f.success,f.message)
        self.assertLess(now,180)
        self.assertLess(math.hypot(x,y),.012)

    def test_face_wall_clock_measurement(self):
        from drawing import Drawing
        from unittest.mock import patch
        with patch("drawing.time.monotonic",return_value=100):
            d=Drawing();d.begin((0,0,0));d.mode="face"
        with patch("drawing.time.monotonic",return_value=245):
            self.assertEqual(d.elapsed,145)
            d.finish(True)
        with patch("drawing.time.monotonic",return_value=1000):
            self.assertEqual(d.elapsed,145)
            self.assertIn("3分以内達成",d.summary)
            d.begin((0,0,0));d.mode="face"
        with patch("drawing.time.monotonic",return_value=1181):
            d.finish(True)
            self.assertIn("3分超過",d.summary)
            d.clear();self.assertEqual(d.elapsed,0)

    def test_face_with_inertia_follows_strokes(self):
        from face import FaceMotion
        for lag in (.1,.25):
            with self.subTest(lag=lag):
                f=FaceMotion((0,0,0),0)
                x=y=yaw=v=w=now=0.;dt=.02;max_cross=0.
                for _ in range(50000):
                    now+=dt
                    cmd_v,cmd_w=f.velocity((x,y,yaw),now,now)
                    v+=(cmd_v-v)*dt/lag;w+=(cmd_w-w)*dt/lag
                    x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
                    if not f.done:
                        cross=-(x-f.start_point[0])*math.sin(f.heading)+(y-f.start_point[1])*math.cos(f.heading)
                        max_cross=max(max_cross,abs(cross))
                    if f.done:break
                self.assertTrue(f.success,f.message)
                self.assertLess(now,150)
                self.assertLess(math.hypot(x,y),.012)
                self.assertLess(max_cross,.015)

    def test_face_waits_for_actual_stop(self):
        from face import FaceMotion
        f=FaceMotion((0,0,0),0)
        f.targets=[(.6,0),(.6,.2)];f._prepare((0,0,0))
        f.velocity((.597,0,0),.1,.1)
        self.assertIsNone(f.arrived)
        self.assertEqual(f.index,0)
        f.velocity((.597,0,0),.2,.2)
        self.assertEqual(f.index,0)
        f.velocity((.597,0,0),.61,.61)
        self.assertEqual(f.index,1)

    def test_compact_face_and_doubled_speed_limits(self):
        from face import FACE, FaceMotion
        self.assertAlmostEqual(max(x for x,y in FACE),.6)
        self.assertAlmostEqual(max(y for x,y in FACE),.75)
        f=FaceMotion((0,0,0),0)
        f.targets=[(.6,0)];f._prepare((0,0,0))
        self.assertEqual(f.velocity((0,0,0),0,0),(0.,0.))
        self.assertEqual(f.velocity((0,0,0),.05,.05),(.8,0.))
        f=FaceMotion((0,0,-math.pi/2),0)
        f.targets=[(0,-.6)];f._prepare((0,0,-math.pi/2))
        self.assertEqual(f.velocity((0,0,0),.05,.05),(0.,-1.4))
        m=Motion();m.update_pose(0,0,0,0);m.start("left",0)
        self.assertEqual(m.velocity(0),(0.,.8))

    def test_no_target_line_is_painted(self):
        from drawing import Drawing, paint
        class Canvas:
            def __init__(self): self.lines=[]
            def winfo_width(self): return 600
            def winfo_height(self): return 320
            def delete(self,*args): pass
            def create_line(self,*args,**kwargs): self.lines.append(kwargs)
            def create_oval(self,*args,**kwargs): pass
            def create_text(self,*args,**kwargs): pass
        for mode in ("square","face","free"):
            d=Drawing();d.begin((0,0,0));d.mode=mode;d.update((.1,0,0))
            c=Canvas();paint(c,d)
            self.assertEqual(len(c.lines),1)
            self.assertNotIn("dash",c.lines[0])
            self.assertEqual(c.lines[0]["fill"],"#1678c8")

    def test_face_language(self):
        ai=Classifier()
        for text in ("ロボットの顔を描いて","ロボットっぽい顔を描いて","ロボットの顔を一筆書きして"):
            self.assertEqual(ai.predict(text)[0],"face")
        self.assertIsNone(ai.predict("ロボットの顔を描かないで")[0])

    def test_face_tracking_and_cancel(self):
        from sequence import SequenceMotion
        m=SequenceMotion(); x,y,yaw,now=2.,-1.,.7,0.;dt=.02
        m.update_pose(x,y,yaw,now);m.start("face",now)
        count=0
        for _ in range(40000):
            now+=dt;m.update_pose(x,y,yaw,now);v,w=m.velocity(now)
            x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
            count+=1
            if m.face is None:break
        self.assertEqual(m.message,"ロボットの顔 完了")
        self.assertTrue(m.drawing.completed)
        self.assertLess(math.hypot(x-2,y+1),.02)
        m.start("face",now);m.stop();self.assertIsNone(m.face)
        self.assertEqual(m.velocity(now),(0.,0.))
        m.start("face",now);self.assertEqual(m.velocity(now+1),(0.,0.))
        self.assertIsNone(m.face);self.assertFalse(m.drawing.completed)

    def test_diagonal_language(self):
        ai=Classifier()
        for side,label in (("右","diagonal_right"),("左","diagonal_left")):
            for text in (side+"斜め前進",side+"斜めに進んで",side+"斜め前に進んでください"):
                self.assertEqual(ai.predict(text)[0],label)
        for text in ("右斜めに進まないで","左斜めに進んで右斜めに進んで"):
            self.assertIsNone(ai.predict(text)[0])

    def test_diagonal_path_and_continuous_drawing(self):
        from sequence import SequenceMotion
        for sign,action in ((1,"diagonal_left"),(-1,"diagonal_right")):
            m=SequenceMotion(); x=y=yaw=now=0.;dt=.02
            m.update_pose(x,y,yaw,now);m.start(action,now)
            for _ in range(2000):
                now+=dt;m.update_pose(x,y,yaw,now);v,w=m.velocity(now)
                x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
                if not m.steps:break
            self.assertIn("完了",m.message)
            self.assertAlmostEqual(yaw,sign*math.pi/4,delta=.02)
            self.assertAlmostEqual(x,.5*math.sqrt(.5),delta=.04)
            self.assertAlmostEqual(y,sign*.5*math.sqrt(.5),delta=.04)
            count=len(m.drawing.points);origin=m.drawing.origin
            m.start("forward",now)
            self.assertEqual(m.drawing.origin,origin)
            self.assertEqual(len(m.drawing.points),count)
            m.stop();self.assertFalse(m.drawing.active)
            m.start(action,now);m.clear_drawing()
            self.assertFalse(m.steps);self.assertFalse(m.drawing.points)
            self.assertEqual(m.velocity(now),(0.,0.))

    def test_drawing_coordinates_measurement_and_abort(self):
        from drawing import Drawing
        d=Drawing(); d.begin((2,3,math.pi/2))
        d.update((2,4,math.pi/2))
        self.assertAlmostEqual(d.points[-1][0],1)
        self.assertAlmostEqual(d.points[-1][1],0)
        self.assertAlmostEqual(d.distance,1)
        d.update((2.03,3.04,0))
        self.assertAlmostEqual(d.gap_cm,5)
        d.finish(True)
        self.assertIn("完了：出発点からのずれ 5.0 cm",d.summary)
        d.update((99,99,0)); self.assertAlmostEqual(d.gap_cm,5)
        d.begin((0,0,0)); d.update((.5,0,0)); d.finish()
        self.assertIn("中断",d.summary)
        self.assertFalse(d.completed)

    def test_drawing_sequence_integration(self):
        from sequence import SequenceMotion
        m=SequenceMotion(); m.start("square",0)
        self.assertFalse(m.drawing.active)
        m.update_pose(0,0,0,1); m.start("square",1)
        self.assertTrue(m.drawing.active)
        m.update_pose(.3,0,0,1.1); m.stop("停止ボタン")
        self.assertFalse(m.drawing.active)
        self.assertFalse(m.drawing.completed)
        self.assertAlmostEqual(m.drawing.gap_cm,30)
        m.start("square",1.1)
        self.assertEqual(m.drawing.points,[(0.,0.)])

    def test_square_language(self):
        ai=Classifier()
        for text in ("四角く一周して", "正方形に走って", "四角を描いて"):
            self.assertEqual(ai.predict(text)[0],"square")
        self.assertIsNone(ai.predict("四角く走らないで")[0])

    def test_square_path_and_abort(self):
        from sequence import SequenceMotion
        m=SequenceMotion(); x=y=yaw=now=0.; dt=.02
        m.update_pose(x,y,yaw,now); m.start("square",now)
        visited=[]
        for _ in range(10000):
            now+=dt
            m.update_pose(x,y,math.atan2(math.sin(yaw),math.cos(yaw)),now)
            v,w=m.velocity(now)
            x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt; yaw+=w*dt
            if m.steps and (not visited or visited[-1]!=m.index): visited.append(m.index)
            if not m.steps: break
        self.assertEqual(visited,list(range(8)))
        self.assertIn("正方形走行完了",m.message)
        self.assertTrue(m.drawing.completed)
        self.assertLess(m.drawing.gap_cm,10)
        self.assertLess(math.hypot(x,y),.1)
        self.assertLess(abs(yaw-2*math.pi),math.radians(3))
        m.start("square",now); m.stop("停止ボタン")
        self.assertEqual(m.velocity(now),(0.,0.)); self.assertFalse(m.steps)
        m.start("square",now)
        self.assertEqual(m.velocity(now+1),(0.,0.)); self.assertFalse(m.steps)
        m.update_pose(x,y,yaw,now+1); m.start("square",now+1)
        m.update_pose(x,y,yaw,now+14)
        self.assertEqual(m.velocity(now+14),(0.,0.)); self.assertFalse(m.steps)

    def test_backward_language(self):
        ai=Classifier()
        for text in ("バック", "バックして", "バックしてください", "後退", "後退して", "後ろに進んで", "後ろに進んでください"):
            with self.subTest(text=text):
                self.assertEqual(ai.predict(text)[0],"backward")

    def test_backward_motion_and_stop(self):
        for heading in (0, math.pi/2, -2.5):
            m=Motion(); m.update_pose(2,3,heading,0); m.start("backward",0)
            self.assertEqual(m.velocity(0),(-.8,0.))
            m.update_pose(2-.45*math.cos(heading),3-.45*math.sin(heading),heading,.1)
            self.assertGreater(m.velocity(.1)[0],-.2)
            self.assertLess(m.velocity(.1)[0],0)
            m.update_pose(2-.49*math.cos(heading),3-.49*math.sin(heading),heading,.2)
            self.assertEqual(m.velocity(.2),(0.,0.))
            self.assertIn("後退完了",m.message)
        m.start("backward",.2); m.stop()
        self.assertEqual(m.velocity(.2),(0.,0.))
        m.start("backward",.2)
        self.assertEqual(m.velocity(.9),(0.,0.))

    def test_feedback_uses_body_pose_not_wheels(self):
        root=ET.parse(Path(__file__).with_name("world.sdf")).getroot()
        model=root.find("world/model[@name='kotoba_robot']")
        plugins=model.findall("plugin")
        publishers=[p for p in plugins if p.findtext("odom_topic")=="/kotoba/odom"]
        self.assertEqual(len(publishers),1)
        self.assertEqual(publishers[0].get("name"),"gz::sim::systems::OdometryPublisher")
        self.assertEqual(publishers[0].findtext("dimensions"),"2")
        drive=model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(drive.findtext("odom_topic"),"/kotoba/wheel_odom")
        bridge=Path(__file__).with_name("bridge.yaml").read_text(encoding="utf-8")
        self.assertIn("gz_topic_name: /kotoba/odom",bridge)

    def test_menu_phrases_regression(self):
        ai=Classifier()
        cases=[("前に進んで","forward"),("前進","forward"),
               ("右を向いて","right"),("左を向いて","left"),("止まって","stop"),
               ("まっすぐ進んでください","forward"),("右に曲がってください","right"),
               ("左に回ってください","left"),("今すぐ止まって","stop"),
               ("明日の天気を教えて",None),("左に曲がって","left")]
        for text,expected in cases:
            with self.subTest(text=text):
                self.assertEqual(ai.predict(text)[0],expected)

    def test_unsupported_phrases_regression(self):
        ai=Classifier()
        for text in ("バックしないで", "後ろに進まないで", "前進してバック", "バックして右を向いて", "バックして360度回転",
                     "前に進まないで", "右を向かないで", "2m進んで", "",
                     "前に進んで右を向いて", "右を向いて前に進んで"):
            with self.subTest(text=text):
                self.assertIsNone(ai.predict(text)[0])

    def test_turn_overshoot_is_corrected(self):
        for action,sign in (("left",1),("right",-1)):
            with self.subTest(action=action):
                m=Motion(); m.update_pose(0,0,0,0); m.start(action,0)
                m.update_pose(0,0,sign*math.radians(94),.2)
                self.assertLess(m.velocity(.2)[1]*sign,0)
                self.assertEqual(m.action,action)
                m.update_pose(0,0,sign*math.radians(90),.3)
                self.assertEqual(m.velocity(.3),(0.,0.))
                self.assertEqual(m.action,action)
                m.update_pose(0,0,sign*math.radians(92),.4)
                self.assertLess(m.velocity(.4)[1]*sign,0)
                m.update_pose(0,0,sign*math.radians(90),.5); m.velocity(.5)
                m.update_pose(0,0,sign*math.radians(90),1.01); m.velocity(1.01)
                self.assertEqual(m.action,"stop")

    def test_slow_approach_and_explicit_stop(self):
        m=Motion(); m.update_pose(0,0,0,0); m.start("left",0)
        m.update_pose(0,0,math.radians(89),.1)
        self.assertGreater(m.velocity(.1)[1],0)
        self.assertLess(m.velocity(.1)[1],.06)
        m.stop(); m.update_pose(0,0,math.radians(94),.2)
        self.assertEqual(m.velocity(.2),(0.,0.))

    def test_lagged_turn_model(self):
        # Synthetic plant with a first-order velocity lag; not a Gazebo result.
        for sign,action in ((1,"left"),(-1,"right")):
            m=Motion(); yaw=0.; rate=0.; dt=.02
            m.update_pose(0,0,0,0); m.start(action,0)
            for i in range(1,1001):
                now=i*dt
                m.update_pose(0,0,yaw,now)
                command=m.velocity(now)[1]
                rate += (command-rate)*dt/.25
                yaw += rate*dt
                if m.action=="stop": break
            self.assertEqual(m.action,"stop")
            self.assertIn("旋回完了",m.message)
            self.assertLess(abs(yaw-sign*math.pi/2),math.radians(1))

    def test_full_turn_language(self):
        ai=Classifier()
        for text in ("360度回転", "３６０度回転", "360度回って", "一周回って", "360度回転してください"):
            with self.subTest(text=text):
                self.assertEqual(ai.predict(text)[0],"spin")
        for text in ("360度回転しないで", "180度回転", "360度回転して前進", "右に360度回転"):
            with self.subTest(text=text):
                self.assertIsNone(ai.predict(text)[0])

    def test_full_turn_multiple_wraps_and_completion(self):
        m=Motion(); start=math.radians(170)
        m.update_pose(0,0,start,0); m.start("spin",0)
        for i in range(1,37):
            yaw=start+math.radians(i*10)
            m.update_pose(0,0,math.atan2(math.sin(yaw),math.cos(yaw)),i*.1)
            m.velocity(i*.1)
            self.assertEqual(m.action,"spin")
        self.assertAlmostEqual(m.angle,2*math.pi)
        m.update_pose(0,0,start,4.11); m.velocity(4.11)
        self.assertEqual(m.action,"stop")
        self.assertIn("+360",m.message)

    def test_full_turn_lagged_plant(self):
        m=Motion(); yaw=0.; rate=0.; dt=.02
        m.update_pose(0,0,0,0); m.start("spin",0)
        for i in range(1,3001):
            now=i*dt
            m.update_pose(0,0,math.atan2(math.sin(yaw),math.cos(yaw)),now)
            command=m.velocity(now)[1]
            rate+=(command-rate)*dt/.25
            yaw+=rate*dt
            if m.action=="stop":break
        self.assertIn("旋回完了",m.message)
        self.assertLess(abs(yaw-2*math.pi),math.radians(1))

    def test_double_forward_speed_and_braking(self):
        m=Motion(); m.update_pose(0,0,0,0); m.start("forward",0)
        self.assertEqual(m.velocity(0),(.8,0.))
        m.update_pose(.45,0,0,.1)
        self.assertGreater(m.velocity(.1)[0],0)
        self.assertLess(m.velocity(.1)[0],.2)
        m.update_pose(.49,0,0,.2)
        self.assertEqual(m.velocity(.2),(0.,0.))

    def test_no_odometry(self):
        m=Motion(); m.start("forward",0)
        self.assertEqual(m.velocity(0),(0.,0.))
    def test_distance_and_stop(self):
        m=Motion(); m.update_pose(0,0,0,0); m.start("forward",0)
        self.assertGreater(m.velocity(.1)[0],0)
        m.update_pose(.49,0,0,.2)
        self.assertEqual(m.velocity(.2),(0.,0.))
    def test_yaw_wrap_both_directions(self):
        for action,sign in (("left",1),("right",-1)):
            m=Motion(); start=sign*math.radians(170)
            m.update_pose(0,0,start,0); m.start(action,0)
            self.assertGreater(m.velocity(.1)[1]*sign,0)
            for step in range(1,10):
                a=start+sign*math.radians(step*10)
                m.update_pose(0,0,math.atan2(math.sin(a),math.cos(a)),step*.04)
            self.assertEqual(m.velocity(.4),(0.,0.))
    def test_stale_timeout_and_cancel(self):
        m=Motion(); m.update_pose(0,0,0,0); m.start("forward",0)
        self.assertEqual(m.velocity(.7),(0.,0.))
        m.update_pose(0,0,0,1); m.start("left",1)
        m.update_pose(0,0,0,27)
        self.assertEqual(m.velocity(27),(0.,0.))
        m.start("forward",27); m.stop()
        self.assertEqual(m.velocity(27),(0.,0.))
    def test_language(self):
        ai=Classifier()
        for text,label in (("前に進んでください","forward"),("右に曲がってください","right"),
                           ("左に回ってください","left"),("今すぐ止まって","stop"),
                           ("前に進まないで",None),("明日の天気を教えて",None)):
            self.assertEqual(ai.predict(text)[0],label)

if __name__=="__main__":
    unittest.main()

