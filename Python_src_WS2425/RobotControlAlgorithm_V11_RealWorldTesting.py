import math
import time
import paho.mqtt.client as mqtt # type: ignore
import json
from datetime import datetime

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.previous_error = 0
        self.integral = 0

    def compute1(self, target, current):
        """Compute PID control output for linear velocity.
        return: linear velocity 
        """
        error = target - current
        # dynamic_kp = self.kp * (abs(error) / (abs(target) + 1e-6))  # 避免除零
        # dynamic_kp = max(dynamic_kp, 2.0) 
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        #return -(self.kp * error + self.ki * self.integral + self.kd * derivative)
        return -(self.kp * error + self.kd * derivative)
    
    def compute2(self, target, current):
        """Compute PID control output.
        return: angular velocity in the range of [-pi, pi]
        """
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        #estimate = self.kp * error + self.ki * self.integral + self.kd * derivative
        estimate = self.kp * error  + self.kd * derivative
        return  (estimate + math.pi) % (2 * math.pi) - math.pi
    
class KalmanFilter:
    """
    Kalman Filter for 1D data.
    todo what does this doß
    """
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance  # 过程噪声
        self.measurement_variance = measurement_variance  # 测量噪声
        self.estimate = initial_value  # 初始估计
        self.error_covariance = 1.0

    def update(self, measurement):
        # 预测阶段
        self.error_covariance += self.process_variance

        # 更新阶段
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error_covariance *= (1 - kalman_gain)

        return self.estimate

class MQTTRobotExecutor:
    
    def __init__(self, broker, broker_fleet_manager, port, order_topic, pose_topic, cmd_topic, state_topic, safety_topic, name):
        self.broker = broker
        self.port = port
        self.order_topic = order_topic
        self.pose_topic = pose_topic
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.safety_topic = safety_topic
        self.broker_fleet_manager = broker_fleet_manager
        self.angular_pid = PIDController(kp=1.0, ki=0.001, kd=0.5)  # 角速度控制
        self.linear_pid = PIDController(kp=2.0, ki=0.001, kd=0.01)  # 线速度控制

        self.kalman_filter_x = KalmanFilter(process_variance=0.1, measurement_variance=0.2)
        self.kalman_filter_y = KalmanFilter(process_variance=0.1, measurement_variance=0.2)
        self.kalman_filter_z = KalmanFilter(process_variance=0.1, measurement_variance=0.2)
        self.kalman_filter_w = KalmanFilter(process_variance=0.1, measurement_variance=0.2)

        self.current_pose = {
            "position": {
                "x": 0.0,  
                "y": 0.0   
            },
            "orientation": {
                "z": 0.0, 
                "w": 1.0  
            }
        } 
        self.waypoints = []  
        self.nodes = []
        self.nodesId = []
        self.sorted_nodesId = []
        self.running = False  
        self.max_linearspeed = 0.4
        self.max_angularspeed = 0.5
        self.max_acceleration = 0.2  
        self.max_angular_acceleration = 0.2  
        self.previous_linear_speed = 0.0
        self.previous_angular_speed = 0.0
        self.safety_state = 0
        self.last_node_id = None
        self.current_passedpoints = 0
        self.velocity = 0

        self.client = mqtt.Client()
        self.client.on_message = self.on_message1
        self.client.on_connect = self.on_connect1

        self.client2 = mqtt.Client()
        self.client2.on_message = self.on_message2
        self.client2.on_connect = self.on_connect2

        #state variables
        self.orderTopicReleased = False
        self.poseTopicCounter = 0
        self.stateHeaderIdIndex = 0
        self.orderId = ""
        self.orderUpdateId = 0
        self.zoneSetId = ""
        #self.lastNodeId = "", this was already defined
        self.lastNodeSequenceId = 0 # the default value could not be read from the files
        #self.driving = False
        self.paused = False  # a function is needed to get the correct value for this variable, not mandatory for the final file
        self.newBaseRequest = False # a function is needed to get the correct value for this variable, not mandatory for the final file
        self.distanceSinceLastNode = "" # a function is needed to get the distance travelled since the last node, not mandatory for the final file
        #self.safetyState = 0, this one is previously defined
        self.agvPosition = {}
        self.poseTopicFlag = True
        self.nodeStates = []
        self.edgeStates = []
        self.actionStates = [] #not currently implemented
        self.remainingWaypoints = []
        self.errors = [] # not currently implemented
        self.edgeStatesFlag = False
        self.name = name
        self.distance = None


    def on_connect1(self, client, userdata, flags, rc):
        """ Callback function for successful connection to MQTT Broker.
            Subscribes to the order topic and pose topic.
            TODO needs rework two clients
        """
        if rc == 0:
            print(f"Connected to MQTT Broker at {self.broker}:{self.port}")
            #client.subscribe(self.order_topic)
            client.subscribe(self.pose_topic)
            client.subscribe(self.safety_topic)
            print(f"Subscribed to topics: {self.pose_topic}, {self.safety_topic}")
        else:
            print(f"Failed to connect, return code {rc}")

    def on_connect2(self, client, userdata, flags, rc):
        """ Callback function for successful connection to MQTT Broker.
            Subscribes to the order topic and pose topic.
            TODO needs rework two clients
        """
        if rc == 0:
            print(f"Connected to MQTT Broker at {self.broker_fleet_manager}:{self.port}")
            client.subscribe(self.order_topic)
            print(f"Subscribed to topics: {self.order_topic}")
        else:
            print(f"Failed to connect, return code {rc}")


    def on_message1(self, client, userdata, msg):
        """Callback function for processing incoming messages from subscribed topics.
        TODO needs rework two clients
        """
        #order_data
        try:
            topic = msg.topic
            message = msg.payload.decode("utf-8")

            if topic == self.safety_topic:
                safety_data = json.loads(message)  
                self.safety_state = safety_data.get("safety_status", -1)
                #print(f"Received Safety State: {self.safety_state}")

            elif topic == self.pose_topic:
                pose_data = json.loads(message)
                pose_data["position"]["x"] = self.kalman_filter_x.update(pose_data["position"]["x"])
                pose_data["position"]["y"] = self.kalman_filter_y.update(pose_data["position"]["y"])
                pose_data["orientation"]["z"] = self.kalman_filter_z.update(pose_data["orientation"]["z"])
                pose_data["orientation"]["w"] = self.kalman_filter_w.update(pose_data["orientation"]["w"])
                self.current_pose = pose_data
                self.poseTopicCounter += 1
                self.agvPosition["x"] = pose_data["position"]["x"]
                self.agvPosition["y"] = pose_data["position"]["y"]
                self.agvPosition["theta"] = math.atan2(
                                            2.0 * (self.current_pose["orientation"]["w"]
                                            * self.current_pose["orientation"]["z"]),
                                            1.0 - 2.0 * (self.current_pose["orientation"]["z"] ** 2)
                                            )
                self.agvPosition["positionInitialized"] = True
                #self.agvPosition["mapId"] = order_data.get("nodes", "")[0].get("mapId", "")
                if self.poseTopicFlag:#and (self.poseTopicCounter % 2 == 0) :
                    self.publish_state()
                    #self.poseTopicFlag = False # This block causes the states to be published the 1st time that pose is read
        except Exception as e:
            print(f"Error processing message from {msg.topic}: {e}")
    
    def on_message2(self, client, userdata, msg):
        """Callback function for processing incoming messages from subscribed topics.
        TODO needs rework two clients
        """
        global order_data
        self.orderTopicReleased = True
        try:
            topic = msg.topic
            message = msg.payload.decode("utf-8")

            if topic == self.order_topic:
                print(f"Received Order Message on {self.name}:", message)
                order_data = json.loads(message)
                nodes = order_data.get("nodes", [])
                self.orderId = order_data.get("orderId", "")
                self.orderUpdateId = order_data.get("orderUpdateId", 0)
                self.waypoints = self.extract_waypoints(nodes)
                self.sorted_nodesId = self.extract_waypointsId(nodes)
                #print("Extracted Waypoints:", self.waypoints)
                self.agvPosition["mapId"] = order_data.get("nodes", [])[0].get("nodePosition", "").get("mapId", "")
                self.poseTopicFlag = False

                self.nodeStates = []
                self.edgeStates = []
                for c1 in range(len(order_data.get("nodes", []))): # c stands for counter
                    nodeState = {}
                    nodeState["nodeId"] = order_data.get("nodes", [])[c1].get("nodeId", "")
                    nodeState["sequenceId"] = order_data.get("nodes", [])[c1].get("sequenceId", "")
                    nodeState["released"] = order_data.get("nodes", [])[c1].get("released")
                    nodeState["nodePosition"] = order_data.get("nodes", [])[c1].get("nodePosition")
                    self.nodeStates.append(nodeState)  
                for c3 in range(len(order_data.get("edges", []))):
                    edgeState = {}
                    edgeState["edgeId"] = order_data.get("edges", [])[c3].get("edgeId", "")
                    edgeState["sequenceId"] = order_data.get("edges", [])[c3].get("sequenceId", 0)
                    edgeState["startNodeId"] = order_data.get("edges", [])[c3].get("startNodeId", "")
                    edgeState["endNodeId"] = order_data.get("edges", [])[c3].get("endNodeId", "")
                    edgeState["actions"] = order_data.get("edges", [])[c3].get("actions", [])
                    self.edgeStates.append(edgeState)
        except Exception as e:
            print(f"Error processing message from {msg.topic}: {e}")
   
    def calibrate_to_initial_pose(self, initial_pose):
        print("Calibrating robot to initial position and orientation...")
    
        while True:
        # Calculate the difference between current position and initial position
            dx = initial_pose["x"] - self.current_pose["position"]["x"]
            dy = initial_pose["y"] - self.current_pose["position"]["y"]
            distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate the angular error for orientation adjustment
            initial_orientation = math.atan2(
                    2.0 * (initial_pose["w"] * initial_pose["z"]),
                    1.0 - 2.0 * (initial_pose["z"] ** 2)
                )
            current_orientation = math.atan2(
                    2.0 * (self.current_pose["orientation"]["w"] * self.current_pose["orientation"]["z"]),
                    1.0 - 2.0 * (self.current_pose["orientation"]["z"] ** 2)
                )
            angular_error = initial_orientation - current_orientation
            angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # First align the orientation
            if abs(angular_error) > 0.05:  # If angular error is significant
               angular_speed = min(self.max_angularspeed, max(-0.5, angular_error))
               self.velocity = 0.0
               self.publish_cmd(self.velocity, angular_speed)
               self.publish_state()
               continue

        # Then adjust the position
            if distance > 0.1:  # If distance to initial position is significant
               linear_speed = min(self.max_linearspeed, max(-self.max_linearspeed, distance))
               angular_speed = 0.0
               self.publish_cmd(linear_speed, angular_speed)
               self.publish_state()
               continue

        # If the robot is aligned and in position, stop
            if distance <= 0.1 and abs(angular_error) <= 0.05:
               self.velocity = 0.0
               self.publish_cmd(self.velocity, 0.0)
               self.publish_state()
               print("Calibration complete. Robot is at initial position and orientation.")
               break


    def extract_waypoints(self, nodes):
        """
        Extracts waypoints from the nodes list and sorts them by their sequence ID.
        """
        released_nodes = [node for node in nodes if node.get("released", False)]
        sorted_nodes = released_nodes #sorted(released_nodes, key=lambda node: node["sequenceId"]), this part is not needed
        waypoints = [{"x": node["nodePosition"]["x"], "y": node["nodePosition"]["y"], "mapId": node["nodePosition"]["mapId"]} for node in sorted_nodes]
        return waypoints
    
    def extract_waypointsId(self, nodes):
        """
        Returns a list of node IDs from the nodes list and sorts them by their sequence ID.
        """
        sorted_nodes = nodes #sorted(nodes, key=lambda node: node["sequenceId"]), this part is not needed
        sorted_nodesId = [node["nodeId"] for node in sorted_nodes]
        return sorted_nodesId
    
    def execute_waypoints(self):
        initial_pose = {"x": self.current_pose["position"]["x"],"y": self.current_pose["position"]["y"],"w": self.current_pose["orientation"]["w"],"z": self.current_pose["orientation"]["z"]}
        self.calibrate_to_initial_pose(initial_pose)

        """
        Main Controll Loop
        """

        # check if there are waypoints available
        if not self.current_pose or not self.waypoints:
            print("No pose or waypoints available for execution.")
            return
                    
        # execute waypoints
        for i, waypoint in enumerate(self.waypoints):
            

            while True:
               #print("Current velocity", self.velocity)
               # compute distance and angle to target
               dx = waypoint["x"] - self.current_pose["position"]["x"]
               dy = waypoint["y"] - self.current_pose["position"]["y"]
               distance = math.sqrt(dx**2 + dy**2)
               self.distance = distance
               angle_to_target = math.atan2(dy, dx)
               current_orientation = math.atan2(
                    2.0 * (self.current_pose["orientation"]["w"] * self.current_pose["orientation"]["z"]),
                    1.0 - 2.0 * (self.current_pose["orientation"]["z"] ** 2)
                )
            
               angular_error = angle_to_target - current_orientation
            
               angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

               angular_speed = self.angular_pid.compute2(angle_to_target, current_orientation)

               linear_speed = self.linear_pid.compute1(0, distance)

               lastnodeposition = {"x": self.waypoints[i-1]["x"], "y": self.waypoints[i-1]["y"]}

               dx1 = self.current_pose["position"]["x"] - lastnodeposition["x"]
               dy1 = self.current_pose["position"]["y"] - lastnodeposition["y"]

               self.distanceSinceLastNode = math.sqrt(dx1**2 + dy1**2)

                #check safety state
               if self.safety_state == 0 :
                    # check if the robot is aligned with the target
                    # rotates first than moves forward

                    # two approaches of closing and turning
                    # first typ: faster but longer distance
                    if abs(angular_error) > math.pi/2:
                       angular_speed = min(self.max_angularspeed, max(-self.max_angularspeed, angular_speed))
                       self.velocity = min(0.01, max(-0.01, linear_speed))
                       self.publish_cmd(self.velocity, angular_speed)  
                       self.publish_state()
                       time.sleep(0.1)

                    elif abs(angular_error) > math.pi/36 and abs(angular_error) < math.pi/2 :
                       angular_speed = min(self.max_angularspeed, max(-self.max_angularspeed, angular_speed))
                       self.velocity = min(0.05, max(-0.05, linear_speed))
                       self.publish_cmd(self.velocity, angular_speed)  
                       self.publish_state()
                       time.sleep(0.1)

                    #moves at full speed for small angular error
                    elif abs(angular_error) < math.pi/36 :
                       angular_speed = min(0.5, max(-0.5, angular_speed))
                       self.velocity = min(self.max_linearspeed, max(-self.max_linearspeed, linear_speed))
                       self.publish_cmd(self.velocity, angular_speed)  
                       self.publish_state()
                       time.sleep(0.1)
                    

                # check if the robot is close to the target
                # check if the waypoint is not the final waypoint
                # initates a turn towards the next waypoint
                #TODO check the distances paramter 1.0 is arbitraty and too high
               if distance < 0.3 and i != len(self.waypoints) - 1: # Distance was previously 0.4
                       '''
                       print(self.waypoints)
                       print(waypoint)
                       print(self.nodes)
                       print(self.nodesId)
                       print(self.sorted_nodesId)
                       print(self.current_passedpoints)
                       '''
                       self.remainingWaypoints = [{"x": wp["x"], "y": wp["y"],"mapId": wp["mapId"]} for wp in self.waypoints[i+1:]]
                       self.last_node_id = f"{self.sorted_nodesId[i]}"
                       self.current_passedpoints += 1
                       if self.orderTopicReleased and self.edgeStatesFlag:
                            self.edgeStates.pop(0)
                            #print(self.edgeStates)
                       if self.orderTopicReleased:
                            self.edgeStatesFlag = True
                            self.lastNodeSequenceId = self.nodeStates[0].get("sequenceId", None)
                            self.nodeStates.pop(0)
                            #print(self.nodeStates)
                            #print(self.last_node_id)
                       self.publish_state()
                       break

            #         #Second typ: accurater but lower speed
            #         if abs(angular_error) > math.pi/2 :
            #            angular_speed = min(self.max_angularspeed, max(-self.max_angularspeed, angular_speed))
            #            self.velocity = 0.0
            #            self.publish_cmd(self.velocity, angular_speed)  
            #            self.publish_state()
            #            time.sleep(0.01)

            #         if abs(angular_error) > math.pi/72 and abs(angular_error) < math.pi/2 :
            #            angular_speed = min(self.max_angularspeed, max(-self.max_angularspeed, angular_speed))
            #            self.velocity = min(0.02, max(-0.02, linear_speed))
            #            self.publish_cmd(self.velocity, angular_speed)  
            #            self.publish_state()
            #            time.sleep(0.01)

            #         #moves at full speed for small angular error
            #         elif abs(angular_error) < math.pi/72 :
            #            angular_speed = min(0.5, max(-0.5, angular_speed))
            #            self.velocity = min(0.7, max(-0.7, linear_speed))
            #            self.publish_cmd(self.velocity, angular_speed)  
            #            self.publish_state()
            #            time.sleep(0.01) 
                    

            #     # check if the robot is close to the target
            #     # check if the waypoint is not the final waypoint
            #     # initates a turn towards the next waypoint
            #     #TODO check the distances paramter 1.0 is arbitraty and to high
            #    if distance < 0.2 and i != len(self.waypoints) - 1:
            #            print(self.waypoints)
            #            print(waypoint)
            #            print(self.nodes)
            #            print(self.nodesId)
            #            print(self.sorted_nodesId)
            #            print(self.current_passedpoints)
            #            self.remainingWaypoints = [{"x": wp["x"], "y": wp["y"],"mapId": wp["mapId"]} for wp in self.waypoints[i+1:]]
            #            self.last_node_id = f"{self.sorted_nodesId[i]}"
            #            self.current_passedpoints += 1
            #            listOfNodes = order_data.get("nodes",[])
            #            for j in range(len(listOfNodes)):
            #                 if listOfNodes[j].get("nodeId") == self.last_node_id:
            #                     self.lastNodeSequenceId = listOfNodes[j].get("sequenceId")
            #            self.publish_state()
            #            break
               
               #check if the robot reached a wapoint
               #TODO Check the Parameter an mak it a CONST in a config file
               if distance < 0.1 and i == len(self.waypoints) - 1:
                       #self.publish_cmd(0.0, 0.0) 
                       self.remainingWaypoints = [{"x": wp["x"], "y": wp["y"], "mapId": wp["mapId"]} for wp in self.waypoints[i+1:]]
                       self.last_node_id = f"{self.sorted_nodesId[i]}"
                       if self.orderTopicReleased and self.edgeStatesFlag:
                            self.edgeStates.pop(0)
                            #print(self.edgeStates)
                       if self.orderTopicReleased:
                            self.edgeStatesFlag = True
                            self.lastNodeSequenceId = self.nodeStates[0].get("sequenceId", None)
                            self.nodeStates.pop(0)
                            #print(self.nodeStates)
                            #print(self.last_node_id)
                       self.edgeStatesFlag = False
                       self.publish_state()
                       break
               
                #check if the robot has collided and stop the robot
               if self.safety_state == 1:                 
                       print(f"Safety state of {self.name} is WARNING. Proceeding cautiously.")
                       self.velocity  = min(0.05, max(-0.05, linear_speed))
                       self.publish_cmd(self.velocity, 0.0)
                       self.running = True
                       continue

               elif self.safety_state == 3:  # 碰撞
                       print(f"Collision detected for {self.name}! Aborting mission.")
                       self.velocity = 0.0
                       self.publish_cmd(self.velocity, 0.0)
                       self.running = False
                       continue
               
                #check if the robot is close to an obstacle and stop the robot
               elif self.safety_state == 2:  # 停止
                       print(f"Safety state of {self.name} is STOP. Halting robot.")
                       self.velocity = 0
                       self.publish_cmd(self.velocity, 0.0)
                       #time.sleep(1.0)
                    #  self.velocity = 0.0
                    #  self.publish_cmd(self.velocity, 0.0)
                       self.running = False
                       continue


               #check if the robot has reached the final waypoint
            if i == len(self.waypoints) - 1:
                    #i = 0
                    lastsecondnodeposition = {"x": self.waypoints[i]["x"], "y": self.waypoints[i]["y"]}
                    dx2 = self.current_pose["position"]["x"] - lastsecondnodeposition["x"]
                    dy2 = self.current_pose["position"]["y"] - lastsecondnodeposition["y"]
                    final_orientation = math.atan2(dy2, dx2)
                    self.waypoints = self.extract_waypoints(self.waypoints) 
                    self.velocity = 0.0
                    self.publish_cmd(self.velocity, 0.0) 
                    self.publish_cmd(self.velocity, 0.0)
                    self.publish_state()
                    
                    current_orientation = math.atan2(
                    2.0 * (self.current_pose["orientation"]["w"] * self.current_pose["orientation"]["z"]),
                    1.0 - 2.0 * (self.current_pose["orientation"]["z"] ** 2)
                    )
                    orientation_error = (final_orientation - current_orientation + math.pi) % (2 * math.pi) - math.pi

                    while abs(orientation_error) > 0.01 and len(self.waypoints) == 0:
                        angular_speed = max(-2.0, min(2.0, 2.0 * orientation_error))  # 比例控制调整角速度
                        self.publish_cmd(0.0, angular_speed)
                        current_orientation = math.atan2(
                        2.0 * (self.current_pose["orientation"]["w"] * self.current_pose["orientation"]["z"]),
                        1.0 - 2.0 * (self.current_pose["orientation"]["z"] ** 2)
                        )
                        orientation_error = (final_orientation - current_orientation + math.pi) % (2 * math.pi) - math.pi

                    self.publish_cmd(self.velocity, 0.0)
                    self.publish_state()
                    self.running = False
                    print("Final waypoint reached. Robot stopped."
                          "Current velocity:", self.velocity)
                    break     

    def constrain_acceleration(self, previous_speed, target_speed, max_acceleration):
        """Constrains the acceleration of the robot to avoid jerky movements."""
        acceleration = target_speed - previous_speed
        if abs(acceleration) > max_acceleration:
            acceleration = max_acceleration if acceleration > 0 else -max_acceleration
        return previous_speed + acceleration

    def publish_cmd(self, linear_x, angular_z):
        """Publishes the linear and angular velocity commands to the robot."""
        linear_x = self.constrain_acceleration(self.previous_linear_speed, linear_x, self.max_acceleration)
        angular_z = self.constrain_acceleration(self.previous_angular_speed, angular_z, self.max_angular_acceleration)

        self.previous_linear_speed = linear_x
        self.previous_angular_speed = angular_z

        cmd_message = {
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        }
        self.client.publish(self.cmd_topic, json.dumps(cmd_message))
        #print("Published Command:", cmd_message)
        if linear_x != 0 or angular_z != 0:
            self.running = True
        else:
            self.running = False

    def publish_state(self):
        """Publishes the current state of the robot.
            TODO add additional information like battery status, etc.
        """
        timestamp = lambda timeString: timeString[:len(timeString)-4] + "Z"
        self.stateHeaderIdIndex += 1

        try:
             self.orderId = order_data.get("orderId", "")
             self.orderUpdateId = order_data.get("orderUpdateId", 0)
        except:
             print(f"order is not published on {self.name}, some parts of state message not complete!")
            
        state_message = {
            "title":"state",
            "description": "all encompassing state of the AGV.",
            "subtopic": "state",
            "type": "object",
            "headerId": self.stateHeaderIdIndex,
            "timestamp": timestamp(datetime.now().isoformat()),
            "version": "3.1.1", # Ask about this version, I am not too sure about this
            "manufacturer": "IFL",
            "serialNumber": "mouse001",
            "orderId": self.orderId,
            "orderUpdateId": self.orderUpdateId,
            "zoneSetId": self.zoneSetId,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.lastNodeSequenceId,
            "driving": self.running,
            "paused": self.paused, 
            "newBaseRequest": self.newBaseRequest,
            "distanceSinceLastNode": self.distanceSinceLastNode,
            "currentPose": self.current_pose,
            "agvPosition": self.agvPosition,
            "remainingWaypoints": self.remainingWaypoints,
            "velocity": self.velocity,
            "safetyState": self.safety_state,
            "nodeStates": self.nodeStates,
            "edgeStates":self.edgeStates,
            "actionStates": self.actionStates,
            "errors": self.errors
        }
        self.client2.publish(self.state_topic, json.dumps(state_message))
        #print("Published State:", state_message)


    def start(self):
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client2.connect(self.broker_fleet_manager, self.port, 60)
            self.client.loop_start()
            self.client2.loop_start()
            print("MQTT client started.")
        except Exception as e:
            print(f"Error starting MQTT client: {e}")

    def stop(self):
        self.client.loop_stop()
        self.client2.loop_stop()
        self.client.disconnect()
        self.client2.disconnect()
        print("MQTT client stopped.")
    
def main():

    MQTT_BROKER_MOUSE = "localhost"
    #MQTT_BROKER_MOUSE = "172.22.222.144"
    FLEET_MANAGEMENT = "localhost"
    #FLEET_MANAGEMENT = "172.22.222.110"
    MQTT_PORT = 1883
    ORDER_TOPIC_MOUSE = "uagv/v2/KIT/mouse001/order"
    POSE_TOPIC_MOUSE = "uagv/v2/KIT/mouse001/pose"
    CMD_TOPIC_MOUSE = "uagv/v2/KIT/mouse001/cmd"
    STATE_TOPIC_MOUSE = "uagv/v2/KIT/mouse001/state"
    SAFETY_TOPIC_MOUSE = "uagv/v2/KIT/mouse001/safety"
    
    MQTT_BROKER_CAT = "localhost"
    #MQTT_BROKER_CAT = "172.22.222.147"
    ORDER_TOPIC_CAT = "uagv/v2/KIT/cat001/order"
    POSE_TOPIC_CAT = "uagv/v2/KIT/cat001/pose"
    CMD_TOPIC_CAT = "uagv/v2/KIT/cat001/cmd"
    STATE_TOPIC_CAT = "uagv/v2/KIT/cat001/state"
    SAFETY_TOPIC_CAT = "uagv/v2/KIT/cat001/safety"
    
    handler1 = MQTTRobotExecutor(MQTT_BROKER_MOUSE, FLEET_MANAGEMENT ,MQTT_PORT, ORDER_TOPIC_MOUSE, POSE_TOPIC_MOUSE, CMD_TOPIC_MOUSE, STATE_TOPIC_MOUSE, SAFETY_TOPIC_MOUSE, "MOUSE")
    handler2 = MQTTRobotExecutor(MQTT_BROKER_CAT, FLEET_MANAGEMENT ,MQTT_PORT, ORDER_TOPIC_CAT, POSE_TOPIC_CAT, CMD_TOPIC_CAT, STATE_TOPIC_CAT, SAFETY_TOPIC_CAT, "CAT")
    handler1.start()
    handler2.start()

    try:
        handler1.publish_state()
        handler2.publish_state()
        while True:
            if handler1.waypoints and not handler1.running:
                handler1.running = True  # 标记机器人正在执行路径
                handler1.execute_waypoints()

            if handler2.waypoints and not handler2.running:
                handler2.running = True
                handler2.execute_waypoints()

    except KeyboardInterrupt:
        print("Stopping the program...")
        handler1.stop()
        handler2.stop()


if __name__ == "__main__":
    main()
