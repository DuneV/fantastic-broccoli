#!/usr/bin/env python3
import rospy
import time 
from pynput import keyboard as kb
from geometry_msgs.msg import Twist
from threading import Thread
from std_msgs.msg import String

class Mov:

    def __init__(self) -> None:
        #Publisher
        self.cmdPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.keypub = rospy.Publisher('teclas', String, queue_size=10)
        print('Ingrese el valor de la velocidad lineal: ')
        self.vel_linear = float(input())
        print('Ingrese el valor de la velocidad angular: ')
        self.vel_angular = float(input())
        print('Robot listo!')

        #Constants
        self.RATE = rospy.Rate(10)

        #Attributes
        self.keyPressed = ""

        thread = Thread(target=self.run)
        thread.start()

    def pressKey(self, key):
        """
        Callback for kb.LIstener: It is called when a key is pressed and depending on the key that is pressed
        a Twist message is published in the cmd_vel topic. Invoke the function 'saveInformation'.
        """
        if key == kb.KeyCode.from_char('w'):
            self.keyPressed = 'w'
        elif key == kb.KeyCode.from_char('q'):
            self.keyPressed = 'q'
        elif key == kb.KeyCode.from_char('s'):
            self.keyPressed = 's'
        elif key == kb.KeyCode.from_char('e'):
            self.keyPressed = 'e'
        elif key == kb.KeyCode.from_char('f'):
            self.keyPressed = 'f'
        else:
            self.keyPressed = ''
    
    def releaseKey(self, key):
        """
        Callback for kb.Listener: It is called when a key is released and a Twist message with all the parameters
        in 0 is published.
        """
        self.keyPressed = ''
    
    def run(self):
        """
        Save in a .txt file the sequence of keys that are pressed.
        """
        """
        Save in a .txt file the sequence of keys that are pressed.
        """
        while not rospy.is_shutdown():
            twistMessage = Twist()
            message = String()
            if self.keyPressed == 'w':
                message = 'w'
                twistMessage.linear.x = self.vel_linear
            elif self.keyPressed == 'q':
                message = 'q'
                twistMessage.angular.z = self.vel_angular
            elif self.keyPressed == 's':
                message = 's'                
                twistMessage.linear.x = -self.vel_linear
            elif self.keyPressed == 'e':
                message = 'e'
                twistMessage.angular.z = -self.vel_angular
            
            self.keypub.publish(message)
            self.cmdPublisher.publish(twistMessage)
            self.RATE.sleep()

    def routine(self):
        arr = ['w','e']
        times = [1,1]
        for i in range(len(times)):
            self.keyPressed = arr[i]
            time.sleep(times[i])


if __name__ == '__main__':    
    rospy.init_node('turtle_bot_teleop')
    punto1 = Mov()
    with kb.Listener(punto1.pressKey, punto1.releaseKey) as escuchador:
        escuchador.join()
    rospy.spin()