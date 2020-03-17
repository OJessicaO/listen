# listen

To set up:

roscore
roslaunch listen recognizer.launch
roslaunch astra_launch astr_pro.launch
rosrun sound_play soundplay_node.py

In catkin_ws:
(to make pyton file executable:
chmod +x blackjack.py
)
In each window:
source ./devel/setup.bash

To run:

roslaunch listen recognizer.launch
python src/listen/src/listener.py
rosrun listen blackjack.py
