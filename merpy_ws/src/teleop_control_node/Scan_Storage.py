import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SimpleTurnPatrol(Node):
    def __init__(self):
        super().__init__('simple_turn_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- CONFIGURATION ---
        
        # 1. SPEED SETTINGS
        # Negative (-) means driving BACKWARDS
        self.LINEAR_SPEED = -0.65   
        
        # Rotation Speed
        # Note: If the robot struggles to turn on floor, increase this value (e.g. to 4.0 or 5.0)
        self.ANGULAR_SPEED_MAGNITUDE = 2.0   
        
        # 2. TIMING & CALIBRATION
        self.PUBLISH_DELAY = 0.02 
        
        # --- CRITICAL CALIBRATION FOR 90 DEGREE TURN ---
        # Adjust this value until the turn is EXACTLY 90 degrees.
        # If it turns too much -> Decrease (e.g., 0.45)
        # If it turns too little -> Increase (e.g., 0.55)
        self.TIME_FOR_90_TURN = 0.4

        # Wait times (in seconds)
        self.WAIT_TIME = 2.0

    def send_cmd(self, linear_x, angular_z, duration):
        """
        Helper: Sends velocity commands continuously for 'duration' seconds.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        start_time = time.time()
        print(f"   >>> ACTUATING: Speed={linear_x}, Turn={angular_z}, Time={duration:.2f}s")

        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            time.sleep(self.PUBLISH_DELAY) 

        # Hard stop after movement to prevent drift
        self.stop_robot(pause_time=0.5)

    def stop_robot(self, pause_time=0.0):
        """ Stops the robot completely """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        # Send stop command multiple times to ensure receipt
        for _ in range(5):
            self.publisher_.publish(msg)
            time.sleep(0.02)
            
        if pause_time > 0:
            print(f"   [Status] Waiting/Stabilizing for {pause_time}s...")
            time.sleep(pause_time)

    # --- MOVEMENT PRIMITIVES ---

    def move_backward_100cm(self):
        """ Drives backward for 100 cm (1.0 meter) """
        target_distance = 0.4 # 100 cm
        
        # Calculate duration = Distance / Speed
        raw_duration = target_distance / abs(self.LINEAR_SPEED)
        
        print(f"--> Moving BACKWARD 100cm (Duration: {raw_duration:.2f}s)")
        self.send_cmd(self.LINEAR_SPEED, 0.0, raw_duration)

    def move_backward_small(self):
        """ Drives backward for a short distance (e.g. 30 cm) """
        target_distance = 0.3 # 30 cm
        
        raw_duration = target_distance / abs(self.LINEAR_SPEED)
        
        print(f"--> Moving BACKWARD Small (30cm) (Duration: {raw_duration:.2f}s)")
        self.send_cmd(self.LINEAR_SPEED, 0.0, raw_duration)

    def turn_right_90(self):
        """ Turns 90 degrees to the RIGHT """
        print("--> Turning RIGHT 90 Degrees")
        # Negative Angular Z = Right Turn
        speed = -1 * abs(self.ANGULAR_SPEED_MAGNITUDE)
        self.send_cmd(0.0, speed, self.TIME_FOR_90_TURN)

    def run_mission(self):
        print("--- MISSION START: STRAIGHT -> TURN -> SHORT DRIVE ---")
        
        # 1. Drive Straight (100cm)
        print("\n[STEP 1] Long Drive")
        self.move_backward_100cm()
        
        # 2. Wait
        self.stop_robot(pause_time=self.WAIT_TIME)

        # 3. Turn Right (90 deg)
        print("\n[STEP 2] 90 Degree Turn")
        self.turn_right_90()

        # 4. Wait
        self.stop_robot(pause_time=self.WAIT_TIME)

        # 5. Drive a little bit more (30cm)
        print("\n[STEP 3] Short Drive")
        self.move_backward_small()

        print("\n--- MISSION COMPLETE. STOPPING. ---")
        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleTurnPatrol()
    
    try:
        print("Waiting 2 seconds for connection stability...")
        time.sleep(2)
        controller.run_mission()
        
    except KeyboardInterrupt:
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()