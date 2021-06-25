from __future__ import print_function

from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import rospy

def generate(request):
    print("Enter generate...")
    return TriggerResponse(success=True)

def add_two_ints_server():
    rospy.init_node('my_ros_python_server')
    s = rospy.Service('generate_random_center', Trigger, generate)
    print("Ready to generate_random_center")
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()