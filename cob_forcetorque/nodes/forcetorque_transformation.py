#!/usr/bin/python
#
#
#
#
#
#
#
#
import roslib
roslib.load_manifest('cob_forcetorque')
import rospy
import tf
from geometry_msgs.msg import *
from std_msgs import *
from tf import TransformListener
from tf import TransformerROS
from tf.transformations import quaternion_from_euler


class Transformation():
	def __init__(self):
		pub = 0
		self.tf = TransformListener()
		self.tf1 = TransformerROS()
		fdata = geometry_msgs.msg.TransformStamped()
		fdata_base = geometry_msgs.msg.TransformStamped()
		transform = tf.TransformBroadcaster()
	
	def wrench_cd(self,msg):
		self.wrench = msg.wrench
		transform.sendTransform((0,0,-0.025),quaternion_from_euler(3.14, 0, 3.665195102, 'rxyz'),rospy.Time.now(),'/ft_debug_link','/arm_7_link')
		fdata.transform.translation.x = self.wrench.force.x
		fdata.transform.translation.y = self.wrench.force.y
		fdata.transform.translation.z = self.wrench.force.z
		
		try:
			if self.tf.frameExists("/dummy_link") and self.tf.frameExists("ft_debug_link"):
				t = self.tf.getLatestCommonTime("/dummy_link", "/ft_debug_link")
				(transform_ee_base_position,transform_ee_base_quaternion) = self.tf.lookupTransform("/dummy_link", '/ft_debug_link', t)
	   				#print transform_ee_base_position
	   				#print transform_ee_base_quaternion
	   				#print self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)
		except(tf.LookupException,tf.ConnectivityException):
			print("TRANSFORMATION ERROR")
			sss.say(["error"])	
			#return 'failed'
		
		fdata_base.transform.translation.x =((self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[0,0] * fdata.transform.translation.x)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[0,1] * fdata.transform.translation.y)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[0,2] * fdata.transform.translation.z)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[0,3]))
		
		fdata_base.transform.translation.y =((self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[1,0] * fdata.transform.translation.x)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[1,1] * fdata.transform.translation.y)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[1,2] * fdata.transform.translation.z)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[1,3]))

		fdata_base.transform.translation.z =((self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[2,0] * fdata.transform.translation.x)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[2,1] * fdata.transform.translation.y)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[2,2] * fdata.transform.translation.z)+ (self.tf1.fromTranslationRotation(transform_ee_base_position,transform_ee_base_quaternion)[2,3]))	
		
		self.wrench_bl.wrench.force.y = fdata_base.transform.translation.y
		self.wrench_bl.wrench.force.x = fdata_base.transform.translation.x
		self.wrench_bl.wrench.force.z = fdata_base.transform.translation.z
		self.wrench_bl.header.stamp = rospy.Time.now();
		self.pub.publish(self.wrench_bl)

		#print ("xxxxx_normal:")
		#print self.wrench.force.x
		#print ("yyyyy_normal")
		#print self.wrench.force.y
		#print ("zzzzz_normal")
		#print self.wrench.force.z
		#print ("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		#print ("xxxxx_trans")
		#print self.wrench_bl.wrench.force.x
		#print ("yyyyy_trans")
		#print self.wrench_bl.wrench.force.y
		#print ("zzzzz_trans")
		#print self.wrench_bl.wrench.force.z
	
	

if (__name__ == "__main__"):
    Trafo = Transformation()
    rospy.init_node('forcetorqueTransformation', anonymous=True)
    rospy.Subscriber("/arm_controller/wrench", WrenchStamped, self.wrench_cb)
    Transformation.pub = rospy.Publisher('/arm_controller/wrench_bl', WrenchStamped)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)		
