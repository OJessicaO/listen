#!/usr/bin/env python
import numpy as np
from collections import defaultdict
import random
from copy import deepcopy

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

PI = 3.1415926535897
mul = 14/9

soundhandle = SoundClient()

# Publish the Twist message to the cmd_vel topic
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
# Publish the screen score display message to the chatter topic
disp_pub = rospy.Publisher('chatter', String, queue_size=10)
# Publish the image display message to the show topic (for start and end screens)
img_pub = rospy.Publisher('show', String, queue_size=10)
cmd_vel = Twist()

# Function to turn the robot towards and away from the center of player group
def rotate(angle):

    speed = 40
    clockwise = False
    angle = angle*mul
    print angle
    # Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    # We wont use linear components
    cmd_vel.linear.x=0

    # Checking if our movement is CW or CCW
    if clockwise:
        cmd_vel.angular.z = -abs(angular_speed)
    else:
        cmd_vel.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        cmd_vel_pub.publish(cmd_vel)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    
    #Forcing our robot to stop
    cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)


# Function to detect person and decide to stay still if that person's turn
def pc_callback(msg):
    
    p0,p1,p2, n = 0, 0, 0, 0
    min_x = -0.02
    max_x = 0.02
    max_depth = 1.2

    for point in point_cloud2.read_points(msg, skip_nans=True):
        # accumulate points only within specified range
        if point[0]<max_x and point[0]>min_x and point[2] < max_depth:
            p0 += point[0]
            p1 += point[1]
            p2 += point[2]
            n += 1

    # if game allows starting
    if(g.start == 1):

        # if enough points and found person to engage, stay still
        if(n>1000 and g.engage==1):
            g.motion = 0
            print(p0/n, p1/n, p2/n)
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            cmd_vel_pub.publish(cmd_vel)
            g.start = 0
            return

        # if no person found, or a current player's turn has ended, turn
        else:
            # update_screen()
            g.motion = 1
            print("No points")
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=-0.4;
            cmd_vel_pub.publish(cmd_vel)

    return

# Class to play game
class Game:
    def __init__(self):
        self.numPlayers = 0
        self.players = []
        self.currentPlayer = -1
        self.dealer = Player()
        self.engage = 1
        self.motion = 1
        self.start = 0

    # function to play the game by creating specified number of player objects
    def play(self):
        img_pub.publish("start")
        rospy.sleep(1)
        soundhandle.playWave('drumroll.wav')
        rospy.sleep(2)
        soundhandle.say("Let's play black jack")
        rospy.sleep(2)
        soundhandle.say("Enter the number of players")
        rospy.sleep(2)
        self.numPlayers =  input("Input the number of players ")

        # create specified number of players
        for n in range(self.numPlayers):
            self.players.append(Player())

        
        soundhandle.playWave('shuffling.wav')
        rospy.sleep(2)
        # deal dealers' first 2 cards
        self.dealer.hit()
        self.dealer.hit()
        
        # deal each player's first 2 cards
        for n in range(self.numPlayers):
            self.players[n].hit()
            self.players[n].hit()
            print "Player ",n," score:",self.players[n].score
        update_screen()
        if self.numPlayers>1:
            rotate(50*(self.numPlayers-1)+10)

        for n in range(self.numPlayers):
            self.engage = 1
            self.currentPlayer = n
            self.start = 1
            #print(self.motion)
            while (self.motion==1):
                rospy.sleep(1)
            #print(self.motion,"..............")
            print "Player",n,"in play"

            # play current player until BUST or stay
            while(True):
                update_screen()
                soundhandle.say("Your score is " + str(self.players[n].score) +" Hit or stay?")

                self.players[n].listen = 1

                # wait until dealer responds to voice input
                while(self.players[n].listen==1):
                    rospy.sleep(1)
        
                # exit turn if player chooses to stay
                if(self.players[n].stay == 1 or self.players[n].score==21):
                    self.players[n].listen = 0
                    soundhandle.say("Your final score is " + str(self.players[n].score))
                    rospy.sleep(3)
                    break

                # exit turn if player's turn has burst
                if(self.players[n].score>21):
                    self.players[n].listen = 0
                    soundhandle.say("Sorry its a BUST!")
                    rospy.sleep(3)
                    print "PLAYER",n,"BURST"
                    break
            update_screen()
            # set engagement for next player if any
            if(n<self.numPlayers-1):
                self.start = 1
                self.engage = 0
                rospy.sleep(6)
                self.engage = 1
        update_screen()
        if self.numPlayers>1:
            rotate(50*(self.numPlayers-1))

        self.start = 0

        # start dealer's turn
        soundhandle.say("Dealer is playing")
        rospy.sleep(3.5)
        print "Dealer's play is going on..."
        while(self.dealer.score<17):
            
            self.dealer.hit()
            update_screen()
            soundhandle.say("Dealer's score is " + str(self.dealer.score))
            rospy.sleep(3)
            if(self.dealer.score>21):
                self.dealer.burst = 1
                print "Dealers score: ",self.dealer.score
                print "DEALER BURST"
                soundhandle.say("Dealer BURST")
                rospy.sleep(3)
                break
            # print "Dealers score: ",self.dealer.score 
            # soundhandle.say("Dealer's final score is" + str(self.dealer.score))
            # rospy.sleep(2)
        update_screen()

        # Display and announce results
        if(self.dealer.burst == 1):
            print "Players won"
            soundhandle.say("The players won. Congratulations!")
            rospy.sleep(4)
            soundhandle.playWave('applause.wav')
            rospy.sleep(2)
        else:
            maxscore = -1
            for n in range(self.numPlayers):
                if(self.players[n].score<22 and self.players[n].score>maxscore):
                    maxscore = self.players[n].score

            result = compare(self.dealer.score,maxscore)
            if(result==2 or result==3):
                update_screen()
                print self.dealer.score
                print "Cheating going on"
                cheat(self.dealer)
                print "After cheating the dealer has ",self.dealer.score
                soundhandle.say("The dealer's final score is " + str(self.dealer.score))
                rospy.sleep(3)
            else:
                update_screen()
                soundhandle.say("The dealer's final score is " + str(self.dealer.score))
                rospy.sleep(3)
                
            for n in range(self.numPlayers):
                update_screen()
                self.currentPlayer = n
                result = compare(self.dealer.score,self.players[n].score)
                announce(self.dealer.score,self.players[n].score,n)
                say(result)
        rospy.sleep(5)
        img_pub.publish("end")
                
        


# class for each player in the game
class Player:
    def __init__(self):
        self.score = 0
        self.cards = []
        self.listen = 0
        self.stay = 0
        self.burst = 0

    # Function to hit the player with another card
    def hit(self):
        card = np.random.choice(deck)
        deck.remove(card)
        self.cards.append(card)
        self.score = self.score + value[card]

# function to announce results based on player and dealer score
def announce(dealerScore, playerScore,num):
    if(playerScore>21 or dealerScore>playerScore):
        soundhandle.say("The dealer won against player " + str(num+1))
        rospy.sleep(3)
        soundhandle.playWave('aww.mp3')
        rospy.sleep(2)
    elif(dealerScore<playerScore):
        soundhandle.say("The player"+str(num+1)+" won against the dealer ")
        rospy.sleep(3)
        soundhandle.playWave('applause.wav')
        rospy.sleep(2)
    elif(dealerScore==playerScore):
        soundhandle.say("The dealer tied with player " + str(num+1))
        rospy.sleep(3)
        soundhandle.playWave('applause.wav')
        rospy.sleep(2)

# function to check comparison of scores 
def compare(dealerScore, playerScore):
    if(playerScore>21 or dealerScore>playerScore):
        return 1
    elif(dealerScore<playerScore):
        return 2
    elif(dealerScore==playerScore):
        return 3

# function for dealer to try and cheat
def cheat(dealer):
    l = deepcopy(deck)
    l.sort()
    val = 0
    for card in l:
        if(dealer.score + value[card]<=21):
            val = value[card]
        else:
            break
    dealer.score = dealer.score + val

# function to print result for given player
def say(i):
    print "Between the dealer and player",g.currentPlayer,":"
    if(i==1):
        print "The dealer won" 
    elif(i==2):
        print "The player won" 
    else:
        print "Tie"



# callback function for voice recognition
def callback(data):

    if(g.currentPlayer>=0 and g.players[g.currentPlayer].listen == 1):
       
        if(data.data=="hit"):
            # hit the player
            g.players[g.currentPlayer].hit()
            print "Player",g.currentPlayer," score: ",g.players[g.currentPlayer].score 
            g.players[g.currentPlayer].listen = 0

        elif(data.data=="stay" or data.data == "stand"):
            # end the current player's turn
            g.players[g.currentPlayer].stay = 1
            g.players[g.currentPlayer].listen = 0

        else:
            # if input not recognized
            soundhandle.say("what?")
            rospy.sleep(1)

    return
          
# function to update screen message being published to the 'chatter' topic
def update_screen():
    s = ""
    for n in range(g.numPlayers):
        for c in g.players[n].cards:
            s = s + str(c) + ','
        s = s + str(g.players[n].score) 
        s = s + ' ' 
    if len(g.dealer.cards) <3:
        s = s + str(g.dealer.cards[0])
    else:
        for c in g.dealer.cards:
            s = s + str(c) + ','
        s = s + str(g.dealer.score)
    print s
    disp_pub.publish(s)

    


if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)

         # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber("/recognizer/output", String, callback)

        # Subscribe to the camera/depth/output topic to detect point clouds.
        rospy.Subscriber('/camera/depth/points', PointCloud2, pc_callback)

        deck = ['A','2','3','4','5','6','7','8','9','J','Q','K'] * 4
        value = defaultdict()
        value['A'] = 1
        value['2'] = 2
        value['3'] = 3
        value['4'] = 4
        value['5'] = 5
        value['6'] = 6
        value['7'] = 7
        value['8'] = 8
        value['9'] = 9
        value['J'] = 10
        value['Q'] = 10
        value['K'] = 10
        
        g = Game()
        g.play()

        rospy.spin()

    except rospy.ROSInterruptException:
      rospy.loginfo("Movement terminated.")

