#!/usr/bin/env python
# coding: latin-1
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Message reçu : %s", data.data)

def subscriber_node():
    # Initialisation du noeud ROS
    rospy.init_node('mon_subscriber', anonymous=True)

    # Crée un abonnement au topic "mon_topic" de type String
    rospy.Subscriber('mon_topic', String, callback)

    # Attend que des messages soient reçus
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass




# #!/usr/bin/env python

# import rospy
# from hello_world.srv import HelloWorld, HelloWorldRequest

# def hello_client():
#     rospy.wait_for_service('hello_world')
#     try:
#         hello_world = rospy.ServiceProxy('hello_world', HelloWorld)
#         request = HelloWorldRequest()
#         response = hello_world(request)
#         return response.message
#     except rospy.ServiceException as e:
#         print("Service call failed:", e)

# if __name__ == "__main__":
#     rospy.init_node('hello_client')
#     result = hello_client()
#     print("Response from server:", result)