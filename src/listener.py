#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#import Tkinter as tk
from Tkinter import *

class FullScreenApp(object):
    def __init__(self, master, **kwargs):
        self.master=master
        pad=3
        self._geom='200x200+0+0'
        master.geometry("{0}x{1}+0+0".format(
            master.winfo_screenwidth()-pad, master.winfo_screenheight()-pad))
        master.bind('<Escape>',self.toggle_geom)            
    def toggle_geom(self,event):
        geom=self.master.winfo_geometry()
        print(geom,self._geom)
        self.master.geometry(self._geom)
        self._geom=geom
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    inp_str = data.data

    root=Tk()
    s = 55
    h = 150

    app=FullScreenApp(root)
    root.configure(bg='black')
    w = Label(root, text="(: (: (: Welcome to BlackJack :) :) :)")
    w.config(font=("Courier", s))
    w.pack()

    num_players = len(inp_str.split())-1

    #Display the heading players
    label1 = Label(root, text="PLAYERS")
    label1.place(x=50, y=h)
    label1.config(font=("Courier", s))
    #Display the number of players
    for pl in range(num_players):
        label1 = Label(root, text="player"+str(pl+1))
        label1.place(x=50, y=h+(pl+1)*100)
        label1.config(font=("Courier", s)) 
    #Display the dealer
    label1 = Label(root, text="Dealer")
    label1.place(x=50, y=h+(num_players+1)*100)
    label1.config(font=("Courier", s))

    #Display the heading cards
    label1 = Label(root, text="CARDS")
    label1.place(x=450, y=h)
    label1.config(font=("Courier", s))
    listt = inp_str.split()
    #Display the cards of players
    for cards in range(len(listt)):
        label1 = Label(root, text=listt[cards][:-len(listt[cards].split(',')[-1])].replace(',',' '))
        label1.place(x=450, y=h+(cards+1)*100)
        label1.config(font=("Courier", s))

    #Display the total sum
    label1 = Label(root, text="SCORE")
    label1.place(x=1100, y=h)
    label1.config(font=("Courier", s))
    #Display the total sum of players
    for cards in range(len(listt)):
        label1 = Label(root, text=listt[cards].split(',')[-1])
        label1.place(x=1100, y=h+(cards+1)*100)
        label1.config(font=("Courier", s))

    if len(listt[-1])==1:
        #Cards
        label1 = Label(root, text=listt[-1]+' ?')
        label1.place(x=450, y=h+(len(listt))*100)
        label1.config(font=("Courier", s))
        #Sum
        label1 = Label(root, text='?')
        label1.place(x=1100, y=h+(len(listt))*100)
        label1.config(font=("Courier", s))

    root.after(3000, root.destroy)
    root.mainloop()

     # root = Tk()
     # app = FullScreenApp(root)
     # root.title("Welcome")
     # lbl = Label(root,text = data.data)
     # lbl.grid(column=0, row=0)
     # root.after(1000, lambda:root.destroy())
     # root.mainloop()
     
     
def listener():
     rospy.init_node('listener', anonymous=True)
     rospy.Subscriber("chatter", String, callback)
     rospy.spin()
 
if __name__ == '__main__':
     listener()

