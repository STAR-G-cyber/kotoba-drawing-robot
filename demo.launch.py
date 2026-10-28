from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    folder = Path(__file__).resolve().parent
    sim = ExecuteProcess(cmd=["gz","sim","-r",str(folder/"world.sdf")],output="screen")
    bridge = Node(package="ros_gz_bridge",executable="parameter_bridge",
                  parameters=[{"config_file":str(folder/"bridge.yaml")}],output="screen")
    app = ExecuteProcess(cmd=["/usr/bin/python3",str(folder/"app.py")],output="screen")
    actions = [SetEnvironmentVariable("ROS_DOMAIN_ID","42"),
               SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST")]
    for process in (sim,bridge,app):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process,on_exit=[EmitEvent(event=Shutdown(reason="Demo process exited"))])))
    return LaunchDescription(actions+[sim,bridge,app])

