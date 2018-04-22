#! /usr/bin/env python

import rospy
import smach
import smach_ros

# define state LIMBO
class LIMBO(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['command_recieved'])
		# TODO initiate all sensors are recievers. Stablish communication with phone

	def execute(self, userdata):
		rospy.loginfo('Executing state LIMBO')
		# TODO once commands are recieved return outcome='command_recieved'
		return 'command_recieved'

# define state LOCAMAP
class LOCAMAP(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['localization_mapping_complete'])
		# TODO start localization scripts
		
	def execute(self, userdata):
		rospy.loginfo('Executing state LOCAMAP')
		# TODO once the robot knows its whereabouts, the destination, 
		# and the route return outcome='localization_mapping_complete'
		return 'localization_mapping_complete'

# define state NAVIGATE
class NAVIGATE(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['destination_reached'])
		# TODO start navigation scripts
		
	def execute(self, userdata):
		rospy.loginfo('Executing state NAVIGATE')
		# TODO once the robot knows its whereabouts, the destination, 
		# and the route return outcome='destination_reached'
		return 'destination_reached'

# define state DELPICK
class DELPICK(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['localization_mapping_complete'])
		# TODO deliver object or recieve it
		
	def execute(self, userdata):
		rospy.loginfo('Executing state DELPICK')
		# TODO a flag will indicate what the next step should be: find safe spot, find drop off spot,
		# or return to sender
		return 'localization_mapping_complete'

# main
def main():
	rospy.init_node('smach_example_state_machine')
	
	# create a SMACH state machine
	sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
	
	# Open the container
	with sm:
		# # Add state to the container
		# smach.StateMachine.add('LIMBO', LIMBO(), 
		# 						transitions={'command_recieved':'LOCAMAP',
		# 									 'localization_mapping_complete':'outcome4'})
											 
		# smach.StateMachine.add('LOCAMAP', LOCAMAP(),
		# 						transitions={'localization_mapping_complete':'LIMBO'})
								
	# Execute SMACH plan
	outcome = sm.execute()
	
if __name__ == '__main__':
	main()
