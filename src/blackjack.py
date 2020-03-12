#!/usr/bin/env python
import numpy as np
from collections import defaultdict
import random
from copy import deepcopy

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

soundhandle = SoundClient()

class Game:
    def __init__(self):
        self.numPlayers = 0
        self.players = []
        self.currentPlayer = -1

    def play(self):
        self.numPlayers =  input("Input the number of players ")
        for n in range(self.numPlayers):
            self.players.append(Player())

        dealer = Player()
        dealer.hit()
        dealer.hit()
        
        for n in range(self.numPlayers):
            self.players[n].hit()
            self.players[n].hit()
            print "Player ",n," score:",self.players[n].score

        for n in range(self.numPlayers):
            self.currentPlayer = n
            print "Player",n,"in play"
            while(True):
                self.players[n].listen = 1
        
                if(self.players[n].stay == 1):
                    self.players[n].listen = 0
                    break
                if(self.players[n].score>21):
                    self.players[n].listen = 0
                    print "PLAYER",n,"BURST"
                    break

        burst = 0
        print "Dealer's play going on..."
        while(dealer.score<17):
            dealer.hit()
            if(dealer.score>21):
                burst = 1
                print "Dealers score: ",dealer.score
                print "DEALER BURST"
                break
            print "Dealers score: ",dealer.score 
        
        if(burst == 1):
            print "Players won"
        else:
            maxscore = -1
            for n in range(self.numPlayers):
                if(self.players[n].score<22 and self.players[n].score>maxscore):
                    maxscore = self.players[n].score

            result = compare(dealer.score,maxscore)
            if(result==2):
                print dealer.score
                print "Cheating going on"
                cheat(dealer)
                print "After cheating the dealer has ",dealer.score
                
            for n in range(self.numPlayers):
                self.currentPlayer = n
                result = compare(dealer.score,self.players[n].score)
                say(result)
                
        



class Player:
    def __init__(self):
        self.score = 0
        self.cards = []
        self.listen = 0
        self.stay = 0

    def hit(self):
        card = np.random.choice(deck)
        deck.remove(card)
        self.cards.append(card)
        self.score = self.score + value[card]

def compare(dealerScore, playerScore):
    if(playerScore>21 or dealerScore>playerScore):
        return 1
    elif(dealerScore<playerScore):
        return 2
    elif(dealerScore==playerScore):
        return 3

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

def say(i):
    print "Between the dealer and player",g.currentPlayer,":"
    if(i==1):
        print "The dealer won" 
    elif(i==2):
        print "The player won" 
    else:
        print "Tie"




def callback(data):

    if(g.currentPlayer>=0 and g.players[g.currentPlayer].listen == 1):
        # rospy.sleep(1)
        # soundhandle.say("I heard " + data.data)
        # rospy.sleep(2)
        if(data.data=="hit"):
            g.players[g.currentPlayer].hit()
            print "Player",g.currentPlayer," score: ",g.players[g.currentPlayer].score 
            g.players[g.currentPlayer].listen = 0
        elif(data.data=="stay" or data.data == "stand"):
            g.players[g.currentPlayer].stay = 1
        else:
            #rospy.sleep(1)
            soundhandle.say("what?")
            rospy.sleep(1)

    return
            # rospy.sleep(1)
            # soundhandle.say("here's your card")
            # rospy.sleep(2)

        
    


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)


    rospy.Subscriber("/recognizer/output", String, callback)

    

    
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



#     sai = Player()
#     dealer = Player()
#     sai.hit()
#     dealer.hit()
#     print("Dealer has a ",dealer.score," and a hidden card")
#     sai.hit()
#     dealer.hit()
#     print("Sai has a score of ",sai.score)
#     while(True):
#         sai.listen = 1
#         # action = input("1.Hit or 2.Stay ")
#         # print("hit or stay")
#         # rospy.sleep(1)
#         # print("outside")

#         # if(action != 1):
#         #     print("breaking")
#         #     break
#         # else:
#         #     sai.hit()
#         #     print("Sais score: ",sai.score)
#         if(sai.stay == 1):
#             sai.listen = 0
#             break
#         if(sai.score>21):
#             sai.listen = 0
#             print("SAI BURST")
#             break
        
#     if(sai.score>21):
#         print('Dealer won')
#     else:
#         burst = 0
#         print("Dealer's play going on...")
#         while(dealer.score<17):
#             dealer.hit()
#             if(dealer.score>21):
#                 burst = 1
#                 print("Dealers score: ",dealer.score)
#                 print("DEALER BURST")
#                 break
#             print("Dealers score: ",dealer.score)
        
#         if(burst == 1):
#             print("Sai won")
#         else:
#             result = compare(dealer.score,sai.score)
#             if(result==1 or result==3):
#                 say(result)
#             else:
#                 print(dealer.score)
#                 print("Cheating going on")
#                 cheat(dealer)
#                 print("After cheating the dealer has ",dealer.score)
#                 result = compare(dealer.score,sai.score)
#                 print("The dealer cheated")
#                 say(result)

# #??? rospy.rate(10)
#     rospy.spin()
