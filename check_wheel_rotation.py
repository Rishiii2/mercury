import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import subprocess

class WheelDiagNode(Node):
    def __init__(self):
        super().__init__('wheel_diag_node')
        self.joint_states = []
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # Publish timer (10 Hz)
        self.timer = self.create_timer(0.1, self.pub_cb)
        self.start_time = time.time()

    def js_cb(self, msg):
        self.joint_states.append(msg)

    def pub_cb(self):
        # Command 1.0 m/s forward speed
        msg = Twist()
        msg.linear.x = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = WheelDiagNode()
    
    print("Starting simulation process in background...")
    # Clean up any existing instances first
    subprocess.run(["pkill", "-9", "gz"])
    subprocess.run(["pkill", "-9", "ruby"])
    subprocess.run(["pkill", "-9", "ros2"])
    time.sleep(2)
    
    # Launch bringup_sim.launch.py
    proc = subprocess.Popen([
        "ros2", "launch", "bringup", "bringup_sim.launch.py", "headless:=true"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    print("Waiting 20 seconds for simulation to start...")
    time.sleep(20)
    
    print("Publishing command and recording joint states for 5 seconds...")
    start_rec = time.time()
    while time.time() - start_rec < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print("Stopping simulation...")
    proc.terminate()
    proc.wait()
    subprocess.run(["pkill", "-9", "gz"])
    subprocess.run(["pkill", "-9", "ruby"])
    
    # Analyze recorded joint states
    if not node.joint_states:
        print("Error: No joint states recorded!")
        return
        
    print("\n--- Diagnostic Results ---")
    latest_state = node.joint_states[-1]
    for name, pos, vel in zip(latest_state.name, latest_state.position, latest_state.velocity):
        if 'wheel' in name or 'suspension' in name:
            print(f"Joint {name}: Position = {pos:.4f} rad, Velocity = {vel:.4f} rad/s")

if __name__ == '__main__':
    main()
