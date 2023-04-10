
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Float64MultiArray

clientAddress = "192.168.0.54"
MQTTHOST = "192.168.0.36"
MQTTPORT = 1883
MQTTTOPIC = "ballon"
mqttClient = mqtt.Client()
pub = rospy.Publisher('openmv', Float64MultiArray, queue_size=1)
rospy.init_node('talker', anonymous=True)
past = rospy.get_rostime().to_sec()-1

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqttClient.subscribe(MQTTTOPIC)

def on_message(client, userdata, msg):
    ballon = str(msg.payload.decode("utf-8"))
    val = [eval(i) for i in ballon.split(' ')]
    #print(round(val[0]/240,2), round(val[1]/160,2), val[2])#240*160
    msg = Float64MultiArray()
    # need data in the format [fx,fy, fz, tx, ty, tz] for bicopter we cant do fy or ty, so we will replace with tz or tx
    horizon = (round(val[0]/240,3)-0.5)
    vertical = (round(val[1]/160,2)-0.5)
    temp = [horizon, vertical, val[2], rospy.get_rostime().to_sec()]
    msg.data = temp
    pub.publish(msg)

if __name__ == '__main__':
    mqttClient.on_connect = on_connect
    mqttClient.on_message = on_message
    mqttClient.connect("test.mosquitto.org", MQTTPORT)
    mqttClient.loop_start()
    while not rospy.is_shutdown():
        pass 