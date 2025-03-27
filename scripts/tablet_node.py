#!/usr/bin/env python
import rospy
import socket
from ast import literal_eval
from std_msgs.msg import *
from misty.srv import *
from misty.msg import *
from misty_reading.msg import *

class TabletSession:
    def __init__(self, host, port):
        # Init node
        rospy.init_node('tablet_node', anonymous=True)
        
        # Init vars
        self.host = host
        self.port = port
        self.correction_indices = []
        self.condition = ''

        # Init pub and sub
        self.tablet_pub = rospy.Publisher('tablet_msg', String, queue_size=10)
        self.word_select_pub = rospy.Publisher('word_selections', Int32MultiArray, queue_size=10)
        self.tablet_status_pub = rospy.Publisher('tablet_status', String, queue_size=10)

        self.correction_sub = rospy.Subscriber('corrections', String, self.correction_callback)
        self.stories_sub = rospy.Subscriber('stories', String, self.story_callback)

    def correction_callback(self, msg):
        # get corrections from manager, send to tablet
        self.correction_indices = msg.data
        returnMessage = "highlight;" + self.correction_indices + "\n"
        self.conn.send(returnMessage.encode())
        print("Correction indices: " + self.correction_indices)
        
    def story_callback(self, msg):
        # get story selections from manager, send to tablet
        self.stories = msg.data
        returnMessage = "story_selection;" + self.stories + "\n"
        self.conn.send(returnMessage.encode())
        print("message sent to tablet: " + returnMessage)

    def run(self):
            # connect to tablet

            BUFFER_SIZE = 1024  

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            print(s.getsockname())
            s.listen(1)
            print('Waiting for client connection...')

            try:
                self.conn, addr = s.accept()
                self.conn.settimeout(None)
                print('Connection address:', addr)
                msg = self.conn.recv(BUFFER_SIZE)
                print(msg)
                returnMessage = "Return connection message"
                self.conn.send(returnMessage.encode())
                print("sent return message")

                msg = String()
                msg.data = "Connected"
                self.tablet_status_pub.publish(msg)
            except KeyboardInterrupt:
                sys.exit()

            sessionEnded = False
                
            while not rospy.is_shutdown():                  # this code handles messages from the tablet

                try:
                    msg = self.conn.recv(BUFFER_SIZE)
                    if not msg: break
                    print("=======================================================================")
                    print("received msg: ", msg)

                    # Parse message
                    msg = msg.decode()
                    msgType = msg.split(";")[0]
                    if len(msg.split(";")) > 1:
                        msgData = msg.split(';')[1]

                    # Actions based on message type
                    if (msgType == 'condition'):
                        if msgData == '0':
                            print("Robot condition selected")
                            self.condition = 'robot'
                            # Send request to management node
                            tab_msg = String()
                            tab_msg.data = "get story"
                            self.tablet_pub.publish(tab_msg)

                        elif msgData == '1':
                            print("Human condition selected")
                            self.condition = 'human'
                            # Inform management node
                            tab_msg = String() 
                            tab_msg.data = "human"
                            self.tablet_pub.publish(tab_msg)

                        else:
                            print('Unexpected result for condition selection')

                    elif (msgType == 'word_selections'):  
                        print("User selected words to highlight")
                        if self.condition == 'human':
                            if msgData:
                                # Send word selections to manager node for human condition
                                msgData = msgData.split(',')
                                word_selects = []
                                for data in msgData: word_selects.append(int(data))
                                print(word_selects)

                                word_selections = Int32MultiArray()
                                word_selections.data = word_selects
                                self.word_select_pub.publish(word_selections)

                            tab_msg = String()
                            tab_msg.data = "get corrections"
                            self.tablet_pub.publish(tab_msg)

                        else:
                            # Send request to management node
                            tab_msg = String()
                            tab_msg.data = "get corrections"
                            self.tablet_pub.publish(tab_msg)
                            print("No user input allowed for Robot condition")

                    elif (msgType == 'next page'):            
                        print("Next page selected")
                        # Get corrections from manager
                        tab_msg = String()
                        tab_msg.data = "next page"

                        # Send request to manager node
                        self.tablet_pub.publish(tab_msg)


                    else:
                        print("Error: unexpected message from tablet")


                except KeyboardInterrupt:
                        self.conn.close()

                        sys.exit(0)

def tablet_message_connection():

    name = socket.gethostname()
    TCP_IP = socket.gethostbyname(str(name))
    print(str(TCP_IP))
    if str(TCP_IP).startswith('127'):
        TCP_IP = socket.gethostbyname(str(name)+".local")
    TCP_PORT = 8080

    session = TabletSession(TCP_IP, TCP_PORT)
    session.run()   

if __name__ == '__main__':
    try:
        tablet_message_connection()
    except rospy.ROSInterruptException:
        pass