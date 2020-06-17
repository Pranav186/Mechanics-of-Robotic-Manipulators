#!	/usr/bin/env	python
import	rospy
import	actionlib
from	practice.msg	import	TimerAction,	TimerGoal,	TimerResult
rospy.init_node('timer_action_client')
client	=	actionlib.SimpleActionClient('timer',	TimerAction)
client.wait_for_server()
goal	=	TimerGoal()
print("type some time input")
t=input()
goal.time_to_wait	=	rospy.Duration.from_sec(t)
client.send_goal(goal)
client.wait_for_result()
print('Time	elapsed:	%f'%(client.get_result().time_elapsed.to_sec()))
