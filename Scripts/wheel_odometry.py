import rospy
from math import sin, cos, pi, atan2

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64

class wheelodom:

    def __init__(self):
        rospy.init_node("wheel_odometry")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started"% self.nodename)

        #PARAMETERS
        self.rate= rospy.get_param('~rate',10.0) #Rate at which to publish
        self.ticks_meter = float(rospy.get_param('ticks_meter',)) #No. of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base width',))#wheel base width

        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') #name of base frame of robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom') #name of the odometry reference frame

        self.encoder_min = rospy.get_param('encoder_min',)
        self.encoder_max = rospy.get_param('encoder_max',)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap',(self.encoder_max - self.encoder_min)*0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap',(self.encoder_max - self.encoder_min)*0.7 +self.encoder_min)

        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now()+ self.t_delta

        #data
        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.lmult =0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0
        self.y = 0
        self.theta= 0
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()

        #subscribed topics
        rospy.Subscriber("lwheel",Int64,self.lwheelCallback)
        rospy.Subscriber("rwheel",Int64,self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom",Odometry,queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
    
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
    
    # transform diff drive to unicycle and using transform to find new position
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            #Odometry
            if self.enc_left = None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left)/ self.ticks_meter
                d_right = (self.right - self.enc_right)/ self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            d = ( d_left + d_right)/2
            theta = (d_right - d_left)/ self.base_width
            self.dx = d / elapsed
            self.dr = theta / elapsed

            if(d!=0):
                x = cos(theta) * d
                y = -sin(theta)* d

                self.x = self.x + ( cos(self.theta)*x - sin(self.theta)*y)
                self.y = self.y + ( sin(self.theta)*x + cos(self.theta)*y)
            if(theta!=0):
                self.theta = self.theta + theta

            #Publisher
            quaternion = Quaternion
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta/2)
            quaternion.w = cos(self.theta/2)
            self.odomBroadcaster.sendTransform(
                (self.x,self.y,0),
                (quaternion.x,quaternion.y,quaternion.z,quaternion.w),
                rospy.Time.now()
                self.base_frame_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x 
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    def lwheelCallback(self, msg):
        enc = msg.data 
        if(enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
        if(enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
        self.left = 1.0 * (enc + self.lmult *(self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc
    
    def rwheelCallback(self,msg):
        enc = msg.data 
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

if if __name__ == "__main__":
    wheel_odometry = Wheelodom()
    wheel_odometry.spin()
