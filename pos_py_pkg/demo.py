# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist 
from tf_transformations import euler_from_quaternion 
from nav_msgs.msg import Odometry


topic1='/model/vehicle_blue/cmd_vel'
topic2='/model/vehicle_blue/odometry'

class ControllerNode(Node):
    def __init__(self,xdu,ydu,Kvu,Kthetau):
        super().__init__('controller_node')
        self.xd=xdu
        self.yd=ydu
        self.Kv=Kvu
        self.Ktheta=Kthetau
        self.lastReceivedMsg=Odometry()
        self.initialTime=time.time()
        self.controlVel=Twist()
        self.controlVel.linear.x=0.0
        self.controlVel.linear.y=0.0
        self.controlVel.linear.z=0.0
        self.controlVel.angular.x=0.0
        self.controlVel.angular.y=0.0
        self.controlVel.angular.z=0.0
        self.ControlPublisher=self.create_publisher(Twist,
                                                    topic1,
                                                    10)
        self.PoseSubscriber=self.create_subscription(Odometry, 
                                                     topic2,
                                                     self.SensorCallback,
                                                     10)
        self.period=0.250
        self.timer=self.create_timer(self.period,
                                     self.ControlFunction)
    def SensorCallback(self,receivedMsg):
        self.lastReceivedMsg=receivedMsg
        self.lastReceivedMsgTime=time.time()

    
    def ControlFunction(self):
        if not hasattr(self, 'lastReceivedMsgTime'):
        # Callback henüz hiç çalışmadıysa, kontrol fonksiyonu çalışmasın
            return
        
        xc = self.lastReceivedMsg.pose.pose.position.x
        yc = self.lastReceivedMsg.pose.pose.position.y
        quat = self.lastReceivedMsg.pose.pose.orientation
        quatl = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = euler_from_quaternion(quatl)
        thetac = yaw

        thetaD = math.atan2((self.yd - yc), (self.xd - xc))
        xvel = self.Kv * math.sqrt((self.xd - xc) ** 2 + (self.yd - yc) ** 2)
        thetavel = self.Ktheta * (thetaD - thetac)

        self.controlVel.linear.x = xvel
        self.controlVel.linear.y = 0.0
        self.controlVel.linear.z = 0.0
        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = thetavel

        print("Sending the control command")
        self.ControlPublisher.publish(self.controlVel)

        timeDiff = self.lastReceivedMsgTime - self.initialTime
        print(f"Time,x,y,theta:({timeDiff:.3f},{xc:.3f},{yc:.3f},{thetac:.3f})")
   

def main(args=None):
    rclpy.init(args=args)
    Kv_u=0.2
    Ktheta_u=0.5
    xd_u=15
    yd_u=15

    TestNode=ControllerNode(xd_u,yd_u,Kv_u,Ktheta_u)
    rclpy.spin(TestNode)
    TestNode.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
