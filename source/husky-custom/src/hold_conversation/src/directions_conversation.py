#!/usr/bin/env python

##############################################
# import libraries
##############################################
import os
import copy
import nltk
from nltk.tree import *
from nltk.parse.stanford import StanfordParser
from nltk.stem.wordnet import WordNetLemmatizer
import nltk_tree_functions as ntf

##############################################
# specify global variables
##############################################
parser      = None
debug       = False
destination = 'something yet to be specified'

##############################################
# define possible responses (and their synonyms)
##############################################
# yes/no
yes          = ['yes', 'sure', 'of course', 'correct', 'yep', 'yup', 'affirmative', 'yeah']
no           = ['no', 'nope', 'do not', "don't", 'negative', 'incorrect', 'nah', 'wrong', 'not']
# start_over
start_over   = ['start over', 'begin again', 'reset', 'beginning']
# directions
turn_around  = ['turn around', 'turn-around', 'backward', 'backwards', 'back', 'behind']
left         = ['left', 'leftward']
right        = ['right', 'rightward']
forward      = ['forward', 'straight', 'down', 'through', 'go to']
either       = ['either']
#follow       = ['follow', 'stay']
# intersection_types
elbow        = ['elbow', 'corner', 'right angle']
three_way    = ['three-way intersection', 'three-way', '3-way', 'three way', 'T-intersection', 'Tee']
four_way     = ['four-way intersection', 'four-way', 'four way']
end          = ['end']
nonspecific  = ['int-L', 'int-R', 'int-F', 'int-B']
# goals
goal         = ['goal', 'destination', 'arrive', 'goal-L', 'goal-R', 'goal-F', 'be there', 'get there', 'over there', 'see it', 'are there']
person       = ['person']

##############################################
# define some primitive_dicts with primitive and their synonyms
# dict[primitive] = [synonym1, synonym2, ...]
##############################################
confirmations = {'yes': yes, 'no': no}
negations     = {'no': no}
start_over    = {'start-over': start_over}
directions    = {'turn-around': turn_around, 'left': left, 'right': right, 'forward': forward, 'either': either} #, 'follow': follow}
intersections = {'elbow': elbow, 'three-way': three_way, 'four-way': four_way, 'nonspecific': nonspecific, 'end': end}
goals         = {'goal': goal, 'person': person}

##############################################
# define some parsing/pos stuff
##############################################
# sentence_split_words
split_words = ['then', '.']
# direction_verbs
direction_verbs = ['go', 'get', 'turn', 'hang', 'take', 'head'] #, 'follow', 'stay']
# destination_verbs 
destination_verbs = ['be', 'find', 'see']
# determiners
determiners = ['first', 'second', 'third', 'fourth', 'fifth', 'last']
# pos tags
phrase_level_pos_tags = ['ADJP', 'ADVP', 'CONJP', 'FRAG', 'INTJ', 'LST', 'NAC', 'NP', 'NX', 'PP', 'PRN', 'PRT', 'QP', 'RRC', 'UCP', 'VP', 'WHADJP', 'WHAVP', 'WHNP', 'WHPP']

##############################################
# get all the valid commands in X up to the first None, then add person
# handle the case where the last command is an intersection
##############################################
def get_valid_X(X):
	# loop through X and keep all the directions up to the first None
	valid_X = []
	for step_idx in range(len(X)):
		if X[step_idx] == None:
			break
		if is_intersection(X[step_idx]) and X[step_idx+1] == None:
			break
		valid_X.append(X[step_idx])
	valid_X.append('person')
	return valid_X

##############################################
# update the subset of X based on the response
##############################################
def update_X(prev_X, X, response, subset, destination):
	# the substring response that we will operate with in this code 
	response_ss = response

	##########
	# PART 0 #
	##########
	# if the user indicated the previous interpretation was wrong, return the prev_X.
	if len(get_all_somethings_in_text(response, negations)) > 0:
		return prev_X, 'negated'
	# if the user indicated the robot should start over, return an empty X.
	if len(get_all_somethings_in_text(response, start_over)) > 0:
		return [None], 'start-over'

	##########
	# PART I #
	##########
	# there is only one query of type "confirmation" (the original question we ask)
	# we look for a 'no' answer.  If there is none, then we proceed to part II.
	if subset['query_type'] == 'confirmation':
		# get all confirmation words
		# result = [idx_in_string, phrase, primitive]
		results = get_all_somethings_in_text(response, confirmations)

		# check if we got an answer
		if len(results) > 0 and subset['template'] == 1 and results[0][2] == 'no':
			# _ -~-> person
			return ['person'], ''

	# if the query_type is "single", we search for the first instance of that answer
	# we then chop off all the words up to and including that word  
	if subset['query_type'] == 'single':
		# get all the instances of the desired answers
		# result = [idx_in_string, phrase, primitive]
		results = get_all_somethings_in_text(response, subset['answers'])

		# check if we got an answer
		if len(results) == 0:
			#print "Did not find a desired answer in the human response"
			return X, ''

		# insert the primitive where it goes (in this subset of X)
		if subset['template'] == 2:
			# _ intersection -~-> direction intersection
			subset['subset'][0] = results[0][2]
		elif subset['template'] == 5:
			# intersection1 _ intersection2 -~-> intersection1 direction intersection2
			subset['subset'][1] = results[0][2]

		# chop off the all the words up to and including the desired answer
		idx = results[0][0] + len(results[0][1])
		response_ss = response_ss[idx:]

		if debug: print '\t\tanswer:     ', results[0][2]
		if debug: print '\t\tsubset:     ', subset['subset']
		if debug: print '\t\tresponse_ss:', response_ss

	###########
	# PART II #
	###########
	# with the remainder of the text, we search on it in an open-ended fashion.
	# we split the sentence up into chunks based on a number of keywords (i.e. split_words)
	# we then search for the presence of certain keywords (directions, intersections, goals)
	# in the case of direction words, we check to see if the describe the goal (by checking to see 
	#	if they are part of the same phrase as the goal).  If they are, then we ignore then.
	# 	If they are not, we assume they are legitimate directions we need to follow.  
	# For each direction, we check to see if they have determiners (e.g. second left).  From the 
	# 	determiners, we can infer certain things (e.g. 'second left' means you will encounter one
	# 	intersection before it with a left turn that you ignore; 'last left' means you will encounter
	# 	N intersections up to an intersection without a forward direction).

	# split the sentence up into chunks based on split_words
	response_chunks = response_ss
	for split_word in split_words:
		response_chunks = response_chunks.replace(split_word, '----')
	response_chunks = response_chunks.split('----')

	# loop over response segments
	for response_chunk in response_chunks:
		# skip over empty chunks
		if response_chunk.strip() == '':
			continue
	
		# run stanford parser + convert iterable that is returned to ParentedTree
		iterable = parser.raw_parse(response_chunk)
		for tree in iterable:
			tree = ParentedTree.convert(tree)
			if debug: tree.pretty_print()

		# get an ordered list of directions + the phrase used to describe the direction.
		instructions = extract_instructions(tree, response_chunk, destination)
		#print 'instructions:', instructions	

		# save off steps
		for instruction in instructions:
			if subset['subset'][-1] == None:
				del subset['subset'][-1]
			subset['subset'] += instruction['robot_cmds']

		if debug: print '\t\tchunk:', subset['subset']

	# TODO: if we had a confirmation or single, does the latter end of our subset match some portion of the 
	# subsequent portion of X[subset['end_idx']:]?  If so, it is likely duplicate and we should ignore it.

	# update X with this subset
	X = X[:subset['start_idx']] + subset['subset'] + X[subset['end_idx']:]
	return X, ''
	
##############################################
# perform inference on X according to some rewrite rules
##############################################
def perform_inference(args, X):
	# check if X contains for certain conditions; if found, make changes to X
	# A)	... intersection1 intersection2 ... -~-> ... intersection1 _ intersection2 ...
	# B)	... direction1 direction2 ... -~-> ... direction1 _ direction2 ...
	# C)	... direction -~-> ... direction _
	# D)	... intersection -~-> ... intersection _
	# E)	... direction forward ... -~-> ... direction ...
	# F)	... goal _ -~-> ... goal
	# G)	... goal ... -~-> ... ...
	# H)	... elbow forward ... -~-> ... elbow ...
	# I)	intersection ... -~-> _ intersection ...
	# J)	intersection_nonspecific direction ... -~-> forward intersection_nonspecific direction ...
	# K)	intersection turn-around ... -~-> turn-around ...
	# L)	turn-around intersection ... -~-> turn-around forward intersection ...
	# M)	... elbow _ ... -~-> ... elbow either ...
	# N)	... intersection goal ... -~-> ... intersection _ goal ...
	# O)	... elbow {int-L, int-R} ... -~-> ... elbow ...
	# R)	goal -~-> _ goal
	# T)	... intersection_specific intersection_nonspecific ... -~-> ... intersection_specific ...
	# U)	... forward forward ... -~-> ... forward ...
	# Y)	_ forward/turn-around ... -~-> forward/turn-around ...
	# NOTE: There is a special case handled in B.  If dir1 is 'turn-around' and dir2 is 'forward', two directions can be adjacent.
	# NOTE: There is a special case handled in E.  If the direction is 'turn-around', forward can follow it.
	# NOTE: There is a special case handled in N.  If the intersection is 'end', goal can follow it.
	# Z) 	if X is longer than num_instructions, truncate it after the last direction, then add 'person' to the end

	'''
	***REMOVED FOLLOW***
	# P)	... intersection follow ... -~-> ... intersection _ follow ...
	# S)	... int-rmViaInfer follow ... -~-> ... follow ...
	# V)	... forward follow ... -~-> ... follow ...
	# NOTE: There is a special case handled in A.  If the direction2 is "follow", two directions can be adjacent.
	# NOTE: There is a special case handled in B.  If the direction1 is "follow", two directions can be adjacent.

	***REMOVED ROUTE CHECKING FOR FIRST DIRECTION***
	# W)	forward ... -~-> left/right ... (when forward is not in list of routes but left/right is) 
	# X)	turn-around ... -~-> left/right ... (when turn-around is not in list of routes but left/right is) 
	'''

	while True:
		X_orig = copy.deepcopy(X)

		# check for R
		if len(X) > 0 and X[0] in goal:
			X.insert(0, None)
		
		# check for F
		if len(X) > 1 and is_goal(X[-2]) and X[-1] == None:
			del X[-1]

		# check for J
		if len(X) > 1 and X[0] in nonspecific and is_direction(X[1]):
			X.insert(0, 'forward')
	
		# check for K
		if len(X) > 1 and is_intersection(X[0]) and X[1] == 'turn-around':
			del X[0]
	
		# check for L
		if len(X) > 1 and X[0] == 'turn-around' and is_intersection(X[1]):
			X.insert(1, 'forward')

		'''
		# check for W and X
		if len(X) > 1 and (X[0] == 'forward' or X[0] == 'turn-around') and not X[0] in routes:
			if 'left' in routes:
				X[0] = 'left'
			elif 'right' in routes:
				X[0] = 'right'
		'''
		
		# check for I
		if len(X) > 0 and is_intersection(X[0]):
			X.insert(0, None)

		'''
		# check for S
		del_idxs = []
		for idx in range(0, len(X)-1):
			if X[idx] == 'int-rmViaInfer' and X[idx+1] == 'follow':
				del_idxs.append(idx)
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]
		'''
		
		# check for U and V
		del_idxs = []
		for idx in range(0, len(X)-1):
			if X[idx] == 'forward' and X[idx+1] == 'forward':
				del_idxs.append(idx)
			'''
			if X[idx] == 'forward' and X[idx+1] == 'follow':
				del_idxs.append(idx)
			'''
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for G
		del_idxs = []
		for idx in range(0, len(X)-1):
			if is_goal(X[idx]):
				del_idxs.append(idx)
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for T
		del_idxs = []
		for idx in range(0, len(X)-1):
			if X[idx] in elbow + three_way + four_way + end and X[idx+1] in nonspecific:
				del_idxs.append(idx+1)
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for E
		del_idxs = []
		for idx in range(0, len(X)-1):
			if not X[idx] == 'turn-around' and is_direction(X[idx]) and X[idx+1] == 'forward':
				del_idxs.append(idx+1)
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for O
		del_idxs = []
		for idx in range(0, len(X)-1):
			if X[idx] == 'elbow' and (X[idx+1] == 'int-R' or X[idx+1] == 'int-L'):
				del_idxs.append(idx+1)
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for A, B, N and P
		insert_idxs = []
		for idx in range(0, len(X)-1):
			if is_intersection(X[idx]) and is_intersection(X[idx+1]): # A
				insert_idxs.append(idx+1)
			if is_intersection(X[idx]) and not X[idx] == 'end' and is_goal(X[idx+1]): # N
				insert_idxs.append(idx+1)
			'''
			if is_intersection(X[idx]) and X[idx+1] == 'follow': # P
				insert_idxs.append(idx+1)
			'''
			if is_direction(X[idx]) and is_direction(X[idx+1]) and not X[idx] == 'turn-around' and not X[idx+1] == 'forward': # and not X[idx+1] == 'follow': # B
				insert_idxs.append(idx+1)
		for idx in range(len(X)-1,-1,-1):
			if idx in insert_idxs:
				X.insert(idx, None)

		# check for C and D
		if is_direction(X[-1]) or is_intersection(X[-1]):
			X.append(None)

		# check for H and M
		del_idxs = []
		for idx in range(0, len(X)-1):
			if X[idx] == 'elbow' and X[idx+1] == 'forward':
				del_idxs.append(idx+1)
			if X[idx] == 'elbow' and X[idx+1] == None:
				X[idx+1] = 'either'
		for idx in range(len(X)-1,-1,-1):
			if idx in del_idxs:
				del X[idx]

		# check for Y
		if len(X) > 2 and X[0] == None and (X[1] == 'forward' or X[1] == 'turn-around'):
			del X[0]

		# check Z
		if len(X) > args['num_instructions']:
			for idx in range(len(X)-1, args['num_instructions']-1, -1):
				del X[idx]
			# discover whether the directions are on even/odd indexes
			last_direction_idx = -1
			for idx in range(len(X)-1,-1,-1):
				if is_direction(X[idx]):
					last_direction_idx = idx
					break
			# truncate accordingly
			if last_direction_idx > 0:
				dir_odd = last_direction_idx % 2 == 0
				ins_odd = args['num_instructions'] % 2 == 0
				if (ins_odd and dir_odd) or (not ins_odd and not dir_odd):
					X[-1]        = 'person'
				elif (ins_odd and not dir_odd) or (not ins_odd and dir_odd):
					del X[-1]
					X[-1]        = 'person'
				else:
					print 'Error!  Unhandled case!'

		# if we've made no changes, break
		if X_orig == X:
			break

	# return updated X
	return X

##############################################
# update the subset of X based on the reponse
##############################################
def all_constraints_satisfied(X):
	# X_0 must be a value from the direction category or goal category
	if not (is_direction(X[0]) or is_goal(X[0])):
		print "invalid X[0]"
		return False

	'''
	# X_0 must be one of the initial directions we can go (unless it is a goal)
	if not X[0] in routes and not is_goal(X[0]):
		print "X[0] not one of the available routes:", routes
		return False
	'''
	# X_n must be a value from the goal category
	if not is_goal(X[-1]):
		print "X[-1] not a goal"
		return False

	# For all X_i, if X_i is of intersection-type, then X_i+1 must be of type direction 
	for i in range(len(X)-1):
		if is_intersection(X[i]) and not is_direction(X[i+1]):
			return False

	# For all X_i, you cannot have 2 values from the category intersection-types beside each other
	for i in range(len(X)-1):
		if is_intersection(X[i]) and is_intersection(X[i+1]):
			return False
	
	# For all X_i, you cannot have 2 values from the category direction beside each other
	# NOTE: There is a special case.  If the first direction is "turn-around", two directions can be adjacent.
	for i in range(len(X)-1):
		if is_direction(X[i]) and is_direction(X[i+1]):
			if i == 0 and X[0] == 'turn-around':
				continue
			return False

	# if you got this far, all the constraints have been satisfied!
	return True


##############################################
# given an instruction, convert to robot commands
##############################################
def get_robot_commands(instruction):
	# initialize commands that we can extract
	cmds = []

	# we first look for the presence of certain key words that will specify the order of events.
	# One of these key words is 'until'.  This indicates you perform the action first, then you
	# encounter the intersection.  The keyword 'at' or 'when' indicate you get to the intersection
	# first and then perform the action.
	# Example:	'go left until you get to the elbow' -> [left, elbow]
	# 			'go left at the elbow' 				 -> [elbow, left] 
	do_action_first = False
	if 'until' in instruction['whole_phrase'] or ('around' in instruction['whole_phrase'] and not 'turn around' in instruction['whole_phrase']):
		do_action_first = True

	# if the direction_det is 'last', then the intersection isn't important
	# you can just return the direction to take when you get to an intersection with no forward
	if instruction['direction_det'] == 'last':
		if do_action_first:
			return [instruction['direction'], 'end']
		else:	
			return ['end', instruction['direction']]

	# if they specified an intersection, we use that...and reason over the intersection_det
	# TODO: we don't take into account phrases like 'turn left at the first two elbows'
	if not instruction['intersection'] == None:
		# determine the number of intersections we go through
		loop_length = 0 
		if instruction['intersection_det'] == 'first':  loop_length = 0;
		if instruction['intersection_det'] == 'second': loop_length = 1;
		if instruction['intersection_det'] == 'third':  loop_length = 2;
		if instruction['intersection_det'] == 'fourth': loop_length = 3;
		if instruction['intersection_det'] == 'fifth':  loop_length = 4;

		# if they used a clause like 'go left until the third intersection'
		if do_action_first:
			cmds += [instruction['direction']]

		# put together the intermediate commands
		for i in range(loop_length):
			cmds += [instruction['intersection'], 'forward']

		# then add the last command
		if instruction['intersection'] == 'end' and instruction['direction'] == 'forward':
			cmds += [instruction['direction'], instruction['intersection']]
		elif do_action_first:
			cmds += [instruction['intersection']]
		else:
			cmds += [instruction['intersection'], instruction['direction']]
		
	# if they did not specify an intersection, we compute a non-specific intersection type
	# we then reason over the direction_det
	else: 
		# determine the intersection type.
		# If they said something like 'take the {first, second, third} left', we know the intersection has a left
		if instruction['direction'] == 'right':
			intersection_type = 'int-R'
		elif instruction['direction'] == 'left':
			intersection_type = 'int-L'
		elif instruction['direction'] == 'forward':
			intersection_type = 'int-F'
		elif instruction['direction'] == 'turn-around':
			intersection_type = 'int-B'
		#elif instruction['direction'] == 'follow':
		#	intersection_type = 'int-rmViaInfer'
		else:
			intersection_type = 'int-dunno!'
		
		# determine the number of intersections we go through
		loop_length = 0 
		if instruction['direction_det'] == 'first':  loop_length = 0;
		if instruction['direction_det'] == 'second': loop_length = 1;
		if instruction['direction_det'] == 'third':  loop_length = 2;
		if instruction['direction_det'] == 'fourth': loop_length = 3;
		if instruction['direction_det'] == 'fifth':  loop_length = 4;

		if do_action_first:
			cmds += [instruction['direction']]
		
		# put together the commands
		for i in range(loop_length):
			cmds += [intersection_type, 'forward']

		# then add the last command
		if instruction['direction'] == 'turn-around':
			cmds += ['turn-around', 'forward']
		elif instruction['direction'] == 'forward':
			cmds += [instruction['direction']]
		elif do_action_first:
			cmds += [intersection_type]
		else:
			cmds += [intersection_type, instruction['direction']]

	# return the commands we have put together		
	return cmds

##############################################
# return the primitive given its synonym
##############################################
def get_primitive_from_synonym(synonym, primitive_dict):
	for primitive, synonyms in primitive_dict.iteritems():
		if synonym in synonyms:
			return primitive
	return None

##############################################
# given a leaf node, get leaf nodes of all (pre-specified) 
# determiners within the closest phrase 
##############################################
def get_determiners(leaf_node):
	phrase_tree = ntf.get_nearest_parent_of_pos(leaf_node, phrase_level_pos_tags)
	# all_leaf_nodes = [['word', leaf_node], ['word', leaf_node], ...]
	all_leaf_nodes = ntf.get_all_leaf_nodes_with_word(ntf.flatten_tree(phrase_tree), determiners)
	result = []
	for leaf_node in all_leaf_nodes:
		result.append(leaf_node[1])
	return result
		
# parse the sentence
# given a parsed sentence tree, pluck out the direction words.
# each direction word will be a keyword preceded by a verb and (possibly) followed by a prepositional phrase
# the direction word may be preceded by a determiner (e.g. "second left")
# the prepositional phrase may contain a reference to an intersection or determiner
# returns the following:
def extract_instructions(tree, text, destination):
	# we keep track of the furthest idx to the right that we process (to make sure we process the entire text)
	furthest_right_idx = 0 

	# get all the leaf nodes (from left to right) that contain a direction word
	# leaf_nodes = [['word', leaf_node], ['word', leaf_node], ...]
	leaf_nodes = ntf.get_all_leaf_nodes_with_word(ntf.flatten_tree(tree), left+right+forward+turn_around) # +follow)

	# create this data structure
	# instructions[idx]['direction']        = {forward, left, right, etc.}
	# instructions[idx]['word']             = 'straight'
	# instructions[idx]['direction_leaf']   = tree_leaf
	# instructions[idx]['idx']              = idx in tree
	instructions = []
	for leaf_node in leaf_nodes:
		direction = get_primitive_from_synonym(leaf_node[0], directions)
		instructions.append({'direction': direction, 'word': leaf_node[0], 'direction_leaf': leaf_node[1], 'idx': leaf_node[2]})
	
	# add some additional data
	# instructions[idx]['verb']             = verb
	# instructions[idx]['phrase_left']      = phrase_left
	# instructions[idx]['phrase_right']     = phrase_right
	# instructions[idx]['whole_phrase']     = whole_phrase
	for instruction in instructions:
		# get verb that precedes this direction
		verb_leaf = ntf.get_earliest_prev_leaf_with_pos(instruction['direction_leaf'], ['VB', 'VBD', 'VBG', 'VBN', 'VBP', 'VBZ'])
		# if that didn't work, try getting the prepositon 
		if verb_leaf == None:
			verb_leaf = ntf.get_earliest_prev_leaf_with_pos(instruction['direction_leaf'], ['IN'])
		# if that didn't work, give up
		if verb_leaf == None:
			verb_leaf = instruction['direction_leaf']

		# get the index (in the tree's leaves) of this verb
		verb_idx  = ntf.get_leaf_idx(tree, verb_leaf, instruction['idx'], 'left')
		
		# there are phrases both before and after the direction_leaf that contain important information,
		# such as intersections, and the presence of words "at", "when" and "until" (which influence
		# whether the direction should happen before or after the intersection).
		# the direction_leaf is sometimes RP, RB, NN, VDB, JJ, VB, IN or other.  
		# we get the nearest ancestor node that is a phrase POS (e.g. PRT, ADVP, NP, VP, ADVP, ADJP).
		# we then get the immediate siblings (left and right).  These siblings are also phrases that 
		# usually contain the important info we are looking for.
		phrase_ancestor_tree = ntf.get_nearest_parent_of_pos(instruction['direction_leaf'], phrase_level_pos_tags)
		phrase_ancestor      = ntf.get_tree_words(phrase_ancestor_tree)
		phrase_left_tree     = None
		phrase_right_tree    = None
		if isinstance(phrase_ancestor_tree, ParentedTree) and isinstance(phrase_ancestor_tree.left_sibling(), ParentedTree):
			phrase_left_tree = phrase_ancestor_tree.left_sibling()
		if isinstance(phrase_ancestor_tree, ParentedTree) and isinstance(phrase_ancestor_tree.right_sibling(), ParentedTree):
			phrase_right_tree = phrase_ancestor_tree.right_sibling()
		# corner case covering the scenario where there are no siblings, but we still want to capture the whole phrase
		if phrase_left_tree == None and phrase_right_tree == None:
			phrase_right_tree = phrase_ancestor_tree

		# it's possible the phrase_left_tree is non-existent or inadequate.  In that case, we replace it with our
		# version of the verb phrase (i.e. from the verb preceding the direction_word to the direction_word)
		# get verb phrase (i.e. only the words between the verb and the direction)
		if isinstance(phrase_left_tree, ParentedTree):
			phrase_left_leaf_nodes = ntf.flatten_tree(phrase_left_tree)
			phrase_left_first_node = phrase_left_leaf_nodes[0]
			phrase_left_start_idx = ntf.get_leaf_idx(tree, phrase_left_first_node, instruction['idx'], 'left')
			if not phrase_left_start_idx == None and not verb_idx == None and verb_idx < phrase_left_start_idx:
				phrase_left_start_node = verb_leaf
				phrase_left_start_idx = verb_idx
			else:
				phrase_left_start_node = phrase_left_first_node
		else:
			phrase_left_start_node = verb_leaf
			phrase_left_start_idx = verb_idx

		# for the right phrase, we just look at the right side (and default to the direction_leaf if there is none)
		if isinstance(phrase_right_tree, ParentedTree):
			phrase_right_last_node = ntf.flatten_tree(phrase_right_tree)[-1]
			phrase_right_start_idx = ntf.get_leaf_idx(tree, phrase_right_last_node, instruction['idx'], 'right')
		else:
			phrase_right_last_node = instruction['direction_leaf']
			phrase_right_start_idx = instruction['idx']

		# keep track of the furthest_right_idx we process
		if phrase_right_start_idx > furthest_right_idx:
			furthest_right_idx = phrase_right_start_idx

		# combine the whole phrase together
		whole_phrase_leaf_nodes = ntf.get_leaf_nodes_between_two_indeces(tree, phrase_left_start_idx, phrase_right_start_idx)
		whole_phrase = ntf.get_words_from_list_of_leaf_nodes(whole_phrase_leaf_nodes)

		# save off additional data
		instruction['verb']                    = ntf.get_tree_words(verb_leaf)
		instruction['whole_phrase']            = whole_phrase
		instruction['whole_phrase_leaf_nodes'] = whole_phrase_leaf_nodes

	# for each direction, determine whether it is an instruction or refers to the destination
	# instructions[idx]['direction_det']    = {'first', 'second', ..., None}
	# instructions[idx]['intersection']     = ParentedTree or None
	# instructions[idx]['intersection_det'] = {'first', 'second', ..., None} 
	# instructions[idx]['robot_cmds']       = [forward, int-L, left, goal-R]
	for instruction in instructions:
		#print 'verb:', WordNetLemmatizer().lemmatize(instruction['verb'],'v')
		
		# check if it is referring a state of being; e.g. 'it will be on your right')
		# TODO: Add in this check to make sure the desination is present in the text
		if WordNetLemmatizer().lemmatize(instruction['verb'],'v') in destination_verbs:
			instruction['direction_det']    = None
			instruction['intersection']     = None
			instruction['intersection_det'] = None
			
			if instruction['direction'] == 'left':
				instruction['robot_cmds'] = ['goal-L']
			elif instruction['direction'] == 'right':
				instruction['robot_cmds'] = ['goal-R']
			elif instruction['direction'] == 'forward':
				instruction['robot_cmds'] = ['goal-F']
			else:
				instruction['robot_cmds'] = ['goal-F']
		
		# check if verb is an action verb (e.g. 'take a left', 'go straight', 'hang a right', etc.)
		else:
			# get any intersections referred to in the 'whole_phrase_tree'
			# TODO: remove assumption that there is at most one intersection in the whole_phrase
			instruction['intersection'] = None
			# intersection_leaf_nodes = [['word', leaf_node], ['word', leaf_node], ...]
			intersection_leaf_nodes = ntf.get_all_leaf_nodes_with_word(instruction['whole_phrase_leaf_nodes'], elbow+three_way+four_way+end)
			if len(intersection_leaf_nodes) > 0:
				instruction['intersection']      = get_primitive_from_synonym(intersection_leaf_nodes[0][0], intersections)
				instruction['intersection_leaf'] = intersection_leaf_nodes[0][1]

			# get any determiners of the intersection (e.g. SECOND three-way intersection, LAST elbow)
			# Assume 'first' if there are no determiners
			instruction['intersection_det']      = 'first'
			instruction['intersection_det_leaf'] = None
			if not instruction['intersection'] == None:
				all_determiners = get_determiners(instruction['intersection_leaf'])
				if len(all_determiners) > 0:
					instruction['intersection_det']      = ntf.get_tree_words(all_determiners[0])
					instruction['intersection_det_leaf'] = all_determiners[0]

			# get any determiners of the direction (e.g. SECOND left, LAST right)
			# Assume 'first' if there are no determiners
			instruction['direction_det'] = 'first'
			all_determiners = get_determiners(instruction['direction_leaf'])
			if len(all_determiners) > 0:
				instruction['direction_det'] = ntf.get_tree_words(all_determiners[0])
			else:
				# we make one more attempt to see if there are determiners in the whole phrase
				# example: turn left at the END of the hallway
				all_determiners = ntf.get_all_leaf_nodes_with_word(instruction['whole_phrase_leaf_nodes'], determiners)
				for determiner in all_determiners:
					# make sure this determiner is not the same one as in the intersection
					if not determiner[1] == instruction['intersection_det_leaf']:
						instruction['direction_det'] = determiner[0]
			
			# get the appropriate set of commands based on the intersection data gathered
			robot_cmds = get_robot_commands(instruction)
			instruction['robot_cmds'] = robot_cmds

	# print debug data
	if debug:
		for instruction in instructions:
			print '\tdirection:',     instruction['direction']
			print '\t\tword             ', instruction['word']
			print '\t\tverb             ', instruction['verb']
			print '\t\twhole_phrase:    ', instruction['whole_phrase']
			print '\t\tdirection:       ', instruction['direction']
			print '\t\tdirection_det:   ', instruction['direction_det']
			print '\t\tintersection:    ', instruction['intersection']
			print '\t\tintersection_det:', instruction['intersection_det']
			print '\t\trobot_cmds:      ', instruction['robot_cmds']

	# TODO: check if there is any remaining text that hasn't been reasoned over 
	if len(leaf_nodes) == 0: 
		unparsed_text_leaf_nodes = ntf.flatten_tree(tree)
		unparsed_text = ntf.get_words_from_list_of_leaf_nodes(unparsed_text_leaf_nodes)
	else:
		last_leaf_idx = len(ntf.flatten_tree(tree))-1
		unparsed_text_leaf_nodes = ntf.get_leaf_nodes_between_two_indeces(tree, furthest_right_idx+1, last_leaf_idx)
		unparsed_text = ntf.get_words_from_list_of_leaf_nodes(unparsed_text_leaf_nodes)
	if debug: print '\t\tunparsed_text:', unparsed_text

	# if you have added no instructions, check to see if there were any intersections in the unparsed_text that you can add in
	all_intersections = get_all_intersections(unparsed_text)
	robot_cmds = []
	for intersection in all_intersections:
		robot_cmds.append(intersection[2])
	if len(robot_cmds) > 0:
		instruction = {'robot_cmds': robot_cmds}
		instructions.append(instruction)
		if debug: print '\t\trobot_cmds int:  ', instruction['robot_cmds']
		
	# check to see if there were any goals in the unparsed_text that you can add at the end
	all_goals = get_all_goals(unparsed_text)
	if len(all_goals) > 0 or destination in text:
		instruction = {'robot_cmds': ['goal-F']}
		instructions.append(instruction)
		if debug: print '\t\trobot_cmds goals:', instruction['robot_cmds']

	# return your accumulated instructions
	return instructions


##############################################
# insert a new record into my data structure 
# data_structure looks like this:
# 	d[idx] = [character_idx, phrase_in_text, primitive]
##############################################
def my_insert(d, new_record):
	# find where the new_record and insert it
	for i in range(len(d)):
		if new_record[0] == d[i][0]:
			# do not insert it because it is a duplicate
			return d
		if new_record[0] < d[i][0]:
			d.insert(i, new_record)
			return d
	# if you didn't insert it in yet, it goes on the end
	d.append(new_record)
	return d

##############################################
# return an ordered list of all 'somethings' found in text
# 'somethings' is a dictionary.  
#	the key is a primitive
#	the value is a list of synonyms
# returned data structure looks like this: 
# 	result[idx] = [character_idx, phrase_in_text, primitive]
##############################################
def get_all_somethings_in_text(text, somethings):
	# create this data structure:
	# result[idx] = [character_idx, phrase_in_text, primitive]
	result = []
	# error check the value is something legit
	if text == None:
		return result
	# remove some punctuation (for comparison below)
	text = ' ' + text.replace(',', '').replace(';', '').replace('.', '') + ' '
	# loop over your somethings
	for primitive, synonyms in somethings.iteritems():
		for synonym in synonyms:
			num_occurrences = text.count(' ' + synonym + ' ') # prepend and append white space so you don't get in-word matches
			substring = text
			last_idx = 0
			for i in range(num_occurrences):
				idx = substring.find(' ' + synonym + ' ')
				result = my_insert(result, [last_idx+idx+1, synonym, primitive])
				substring = substring[idx+len(synonym):]
				last_idx = last_idx + idx
	# return all instances you found (in order they appear in text)
	return result

##############################################
# return whether the text contains a direction 
##############################################
def is_direction(text):
	if len(get_all_somethings_in_text(text, directions)) == 0:
		return False
	else:
		return True

##############################################
# return whether the text contains an intersection 
##############################################
def is_intersection(text):
	if len(get_all_somethings_in_text(text, intersections)) == 0:
		return False
	else:
		return True

##############################################
# return whether the text contains a goal 
##############################################
def is_goal(text):
	if len(get_all_somethings_in_text(text, goals)) == 0:
		return False
	else:
		return True

##############################################
# return whether the text contains a confirmation 
##############################################
def is_confirmation(text):
	if len(get_all_somethings_in_text(text, confirmations)) == 0:
		return False
	else:
		return True

##############################################
# return all directions found in the text (in order)
##############################################
def get_all_directions(text):
	return get_all_somethings_in_text(text, directions)

##############################################
# return all intersections found in the text (in order)
##############################################
def get_all_intersections(text):
	return get_all_somethings_in_text(text, intersections)

##############################################
# return all goals found in the text (in order)
##############################################
def get_all_goals(text):
	return get_all_somethings_in_text(text, goals)

##############################################
# return all confirmations found in the text (in order)
##############################################
def get_all_confirmations(text):
	return get_all_somethings_in_text(text, confirmations)

##############################################
# return the indeces on the left/right of an idx that are not None
##############################################
def get_neighboring_known_indeces(X, idx):
	# find the left idx
	start_idx = 0
	for i in range(idx-1, -1, -1):
		if not X[i] == None:
			start_idx = i
			break

	# find the right idx
	end_idx = len(X)
	for i in range(idx, len(X)):
		if not X[i] == None:
			end_idx = i+1
			break

	# return indeces
	return [start_idx, end_idx]	

##############################################
# merge dictionaries
##############################################
def merge_dicts(list_of_dicts):
	# start with the first dicts keys and values
	merged_dict = list_of_dicts[0].copy()
	# loop over the remaining dicts
	for i in range(1, len(list_of_dicts)):
		# modifies merged_dict with list_of_dicts[i]'s keys and values & returns None
		merged_dict.update(list_of_dicts[i])	
	return merged_dict

##############################################
# generate a query based on X
# This is based, in part, on frame-based dialog.
# See here: https://web.stanford.edu/~jurafsky/slp3/24.pdf
##############################################
def generate_query(X, destination):
	# create this data structure:
	# subsets[id]['start_idx']  = start_idx
	# subsets[id]['end_idx']    = end_idx
	# subsets[id]['subset']     = [knownValue _ knownValue]
	# subsets[id]['query']      = query to ask about this subset of X
	# subsets[id]['query_type'] = {'confirmation', 'single', 'open-ended'}
	# subsets[id]['answers']    = list of strings of responses we are looking for (e.g. direction, intersection, goal) 
	subsets = []

	# from X, find get each subset containing an unknown variables
	# populate subsets with start_idx, end_idx, subset, and phrase
	for idx in range(len(X)):
		if X[idx] == None:
			start_idx, end_idx = get_neighboring_known_indeces(X, idx)
			this_dict = {'start_idx': start_idx, 'end_idx': end_idx, 'subset': X[start_idx:end_idx]}
			subsets.append(this_dict)

	# for each each subset, match it to templates; then poplate its query, query_type, and answer
	for idx in range(len(subsets)):
		# template 1:	pattern = '_'
		if len(subsets[idx]['subset']) == 1:
			subsets[idx]['template']   = 1
			subsets[idx]['query']      = 'Could you tell me how to navigate to ' + destination + '?'
			subsets[idx]['query_type'] = 'confirmation'
			subsets[idx]['answers']    = confirmations

		# template 2:	pattern = '_ intersection' or '_ goal'
		elif len(subsets[idx]['subset']) == 2 and (is_intersection(subsets[idx]['subset'][1]) or is_goal(subsets[idx]['subset'][1])):
			subsets[idx]['template']   = 2
			subsets[idx]['query']      = 'Which direction do I start out going?'
			subsets[idx]['query_type'] = 'single'
			subsets[idx]['answers']    = directions

		# template 3:	pattern = 'direction _'
		elif len(subsets[idx]['subset']) == 2 and is_direction(subsets[idx]['subset'][0]):
			direction = subsets[idx]['subset'][0]

			# if this is the start of the X:
			if subsets[idx]['start_idx'] == 0 and X[0] == 'turn-around':
				query = 'What do I do after turning around?'
			elif subsets[idx]['start_idx'] == 0:
				query = 'What do I do after starting to go ' + direction + '?'
			elif subsets[idx]['start_idx'] == 1:
				query = 'What do I do after turning around and going ' + direction + '?'
			# else there are N directions they have taken up to this point
			else:
				n_directions = X[0:subsets[idx]['start_idx']+1].count(subsets[idx]['subset'][0])
				if n_directions == 1: nth = 'first'
				if n_directions == 2: nth = 'second'
				if n_directions == 3: nth = 'third'
				if n_directions == 4: nth = 'fourth'
				if n_directions == 5: nth = 'fifth'

				intersection = X[subsets[idx]['start_idx']-1]
				n_intersections = X[0:subsets[idx]['start_idx']+1].count(intersection)
				if n_intersections == 1: jth = 'first'
				if n_intersections == 2: jth = 'second'
				if n_intersections == 3: jth = 'third'
				if n_intersections == 4: jth = 'fourth'
				if n_intersections == 5: jth = 'fifth'

				if subsets[idx]['subset'][0] == 'left' or subsets[idx]['subset'][0] == 'right':
					query = 'What do I do after I turn ' + direction
					if nth == 'first': query += "?"
					else:			   query += ' (this being the ' + nth + ' ' + direction + ' I take)?'
				elif intersection == 'elbow':
					query = 'Where do I go after the elbow (this being the ' + jth + ' elbow)?'
				else:
					query = 'What do I do after going through the ' + jth + ' ' + intersection + '?'

			subsets[idx]['template']   = 3
			subsets[idx]['query']      = query
			subsets[idx]['query_type'] = 'open-ended'
			subsets[idx]['answers']    = merge_dicts([intersections, directions, goals])

		# template 4:	pattern = 'intersection _'
		elif len(subsets[idx]['subset']) == 2 and is_intersection(subsets[idx]['subset'][0]):
			intersection = subsets[idx]['subset'][0]

			n_intersections = X[0:subsets[idx]['start_idx']+1].count(intersection)
			jth = ''
			if n_intersections == 1: jth = 'first'
			if n_intersections == 2: jth = 'second'
			if n_intersections == 3: jth = 'third'
			if n_intersections == 4: jth = 'fourth'
			if n_intersections == 5: jth = 'fifth'

			if intersection == 'int-F':      intersection = 'intersection where I can go forward'
			elif intersection == 'int-R':    intersection = 'intersection where I can go right'
			elif intersection == 'int-L':    intersection = 'intersection where I can go left'
			elif intersection == 'end':      intersection = 'end of the hall'
			
			subsets[idx]['template']   = 4
			subsets[idx]['query']      = 'What do I do after getting to the ' + jth + ' ' + intersection + '?'
			subsets[idx]['query_type'] = 'open-ended'
			subsets[idx]['answers']    = merge_dicts([intersections, directions, goals])

		# template 5:	pattern = 'intersection1 _ intersection2'
		elif is_intersection(subsets[idx]['subset'][0]) and is_intersection(subsets[idx]['subset'][2]):
			intersection1 = subsets[idx]['subset'][0]
			intersection2 = subsets[idx]['subset'][2]
			subsets[idx]['template']   = 5
			subsets[idx]['query']      = 'When I am at ' + intersection1 + ', which direction will I go to get to ' + intersection2 + '?'
			subsets[idx]['query_type'] = 'single'
			subsets[idx]['answers']    = directions

		# template -1:	pattern = something else!
		else:
			X = [None]
			subsets = [{'start_idx': 0, 'end_idx': 1, 'subset': [None]}]
			subsets[0]['template']     = 1
			subsets[0]['query']        = 'Sorry, I am a little confused.  Could we start over?  Could you tell me how to navigate to ' + destination + '?'
			subsets[0]['query_type']   = 'confirmation'
			subsets[0]['answers']      = confirmations
			break

	# return queries
	return [subsets, X]

##############################################
# summarize set of steps.
##############################################
def summarize(X, destination):
	# keep track of the summary and the number of remaining_steps
	summary = 'Let me summarize in my own words.  '
	remaining_steps = len(X)

	# handle special case of one instruction
	if len(X) == 1:
		if X[0] == 'person':
			return summary + 'I will look for someone else to help me.'
		else:
			return summary + 'I am in the same hallway as ' + destination + '.  I just need to look for it!'

	# handle special case of two instructions
	if len(X) == 2:
		return summary + 'I go ' + X[0] + ' until I find ' + destination + '.'

	# if the first instruction is turn-around, handle it.  Otherwise, start out with standard beginning
	if X[0] == 'turn-around':
		if len(X) == 3:
			if X[2] == 'goal-F':
				return summary + 'I turn around and go ' + X[1] + ' until I get to ' + destination + '.'
			elif X[2] == 'goal-L':
				return summary + 'I turn around and go ' + X[1] + ' until I get to ' + destination + ' on my left.'
			elif X[2] == 'goal-R':
				return summary + 'I turn around and go ' + X[1] + ' until I get to ' + destination + ' on my right.'
		else:
			summary += 'I turn around and go ' + X[1] + '.  '
			remaining_steps -= 2
	else:
		summary += 'I start out going ' + X[0] + '.  '
		remaining_steps -= 1

	# loop through the remaining steps and summarize them
	while remaining_steps > 0:
		idx = len(X) - remaining_steps

		# Wrap things up.
		if remaining_steps == 1:
			if X[idx] == 'goal-F':
				summary += 'Then ' + destination + ' will be somewhere up ahead.'
			elif X[idx] == 'goal-L':
				summary += 'Then ' + destination + ' will be on my left.'
			elif X[idx] == 'goal-R':
				summary += 'Then ' + destination + ' will be on my right.'
			elif X[idx] == 'person':
				summary += 'Then, because my memory is only so-so, I will look for another person to ask for help.'
			return summary

		# if the intersection is elbow and the user didn't specify the direction, say so
		elif X[idx] == 'elbow' and X[idx+1] == 'either':
			summary += 'I go past the elbow.  '
			remaining_steps -= 2

		# if the next step is a follow, say so
		#elif X[idx] == 'follow':
		#	summary += 'I follow this hallway.  ' 
		#	remaining_steps -= 1
		
		# if the intersection is the last one in a hall, say so
		elif X[idx] == 'end':
			summary += 'I go ' + X[idx+1] + ' when I get to the end of the hall.  '
			remaining_steps -= 2
		
		# if the intersection is non-specific, we look forward until we find a left/right direction
		elif X[idx] in nonspecific:
			i = 1
			while (idx+i)<len(X)-1:
				#print 'idx:', idx, 'i:', i, 'len(X):', len(X), 'X[idx+i]:', X[idx+i]
				if X[idx+i] in left + right or X[idx+i+1] in goal + person:
					remaining_steps -= i+1
					break
				i += 2
			if i/2 == 0: summary += 'I take my first '  + X[idx+i] + '.  '
			if i/2 == 1: summary += 'I take my second ' + X[idx+i] + '.  '
			if i/2 == 2: summary += 'I take my third '  + X[idx+i] + '.  '
			if i/2 == 3: summary += 'I take my fourth ' + X[idx+i] + '.  '
			if i/2 == 4: summary += 'I take my fifth '  + X[idx+i] + '.  '
		
		# if the intersection is standard, add a standard step summary
		else:
			summary += 'When I get to a ' + X[idx] + ', I go ' + X[idx+1] + '.  '
			remaining_steps -= 2

	
	return summary + '.  should not have got here'

##############################################
# setup the parser stuff
##############################################
def setup(args):
	global parser, debug
	os.environ['STANFORD_PARSER'] = args['parser']
	os.environ['STANFORD_MODELS'] = args['models'] 
	parser=StanfordParser(model_path=args['model_path'])
	debug = args['debug']

##############################################
# small helper function to test an input sample 
##############################################
def test_sample(input_text, desired, destination, start_X=[None], subset_input={}):
	print '\n========================================================\n'
	print input_text
	if subset_input == {}:
		subset = {'query_type': 'confirmation', 'subset': [None], 'start_idx': 0, 'end_idx': 1, 'answers': confirmations, 'template': 1}
	else:
		subset = subset_input
	X, message = update_X([None], start_X, input_text, subset, destination)
	X = perform_inference(args, X)
	passed = True if X == desired else False
	print '====RESULTS===='
	print 'desired:', desired
	print 'actual: ', X
	print 'passed: ', passed 

##############################################
# run a small suite of tests
##############################################
def run_test_samples():
	input_text = 'head down this hallway, take a left when you get to the corner, then go to the end of the hallway hang a left and you will be there'
	desired = ['forward', 'elbow', 'left', 'end', 'left', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'turn around, head down the hallway then go left'
	desired = ['turn-around', 'forward', 'int-L', 'left', None]
	test_sample(input_text, desired, 'the staircase')

	input_text = 'you go down this hallway, then hang a left, then go further, hang another left, and you are there'
	desired = ['forward', 'int-L', 'left', 'int-L', 'left', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'go down this hallway follow it around the corner and just keep going until you get there'
	desired = ['forward', 'elbow', 'either', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'go straight until you hit a three-way then go right until you hit a four-way go through it then turn left'
	desired = ['forward', 'three-way', 'right', 'four-way', 'forward', 'int-L', 'left', None]
	test_sample(input_text, desired, 'the staircase')

	input_text = 'go straight and take a left at the end of the hall. The bathroom will be at the end on the right'
	desired = ['forward', 'end', 'left', 'goal-R']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'you keep going straight until you hit an elbow go right then take your second left.'
	desired = ['forward', 'elbow', 'right', 'int-L', 'forward', 'int-L', 'left', None]
	test_sample(input_text, desired, 'the staircase')

	input_text = 'when you get to the end of the hall turn right then your destination will be on your left'
	desired = [None, 'end', 'right', 'goal-L']
	test_sample(input_text, desired, 'the staircase')

	#input_text = "just follow the hallway and you'll see it"
	#desired = ['follow', 'goal-F']
	#test_sample(input_text, desired, 'the staircase')

	input_text = "yeah, it's right over there"
	desired = [None, 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = "if you go down the hallway, past the men's bathroom and through the archway, it'll be on your right"
	desired = ['forward', None, 'goal-R']
	test_sample(input_text, desired, 'the staircase')

	#input_text = "go left, follow the hallway until you get to a three-way, then turn right and the staircase will be on your right"
	#desired = ['left', 'follow', 'three-way', 'right', 'goal-R']
	#test_sample(input_text, desired, 'the staircase')

	#input_text = "stay in this hallway until you get to a four-way, turn left and then you will see it"
	#desired = ['follow', 'four-way', 'left', 'goal-F']
	#test_sample(input_text, desired, 'the staircase')
	
	input_text = 'go ahead, and then at the corner turn left and go ahead you will find staircase at your right side.'
	desired = ['forward', 'elbow', 'left', 'goal-R']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'turn right and then keep going forward until reach the corner and then turn left.  After that you go forward and at the end of the road you will find the staircase.'
	desired = ['forward', 'int-R', 'right', 'elbow', 'left', 'end', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'Yes.  You first go backward and then at the corner you will find the staircase.'
	desired = ['turn-around', 'forward', 'elbow', 'either', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = "Sorry, I don't know."
	desired = ['person']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'Alright.  Turn left and go ahead then at your right side you will find a door.   Open the door enter the room you will find the staircase there.'
	desired = ['forward', 'int-L', 'left', 'int-R', 'right', 'goal-F']
	test_sample(input_text, desired, 'the staircase')

	input_text = 'Take your second left.'
	desired = ['forward', 'int-L', 'forward', 'int-L', 'left', None]
	test_sample(input_text, desired, 'the staircase')

	start_X = [None, 'end', 'left', None]
	subset_input = {}
	subset_input['start_idx']  = 0
	subset_input['end_idx']    = 1
	subset_input['subset']     = [None]
	subset_input['query']      = 'Which direction do I start out going?'
	subset_input['query_type'] = 'single'
	subset_input['answers']    = directions
	subset_input['template']   = 2
	input_text = 'turn-around'
	desired = ['turn-around', 'forward', 'end', 'left', None]
	test_sample(input_text, desired, 'the staircase', start_X, subset_input)


##############################################
# if calling from the command line
##############################################
if __name__ == '__main__':
	args = {}
	args['parser']           = "/home/jjohanse/nltk_data/stanford-parser-full-2018-10-17/"
	args['models']           = "/home/jjohanse/nltk_data/stanford-parser-full-2018-10-17/stanford-parser-3.9.2-models.jar"
	args['model_path']       = "/home/jjohanse/nltk_data/stanford-parser-full-2018-10-17/edu/stanford/nlp/models/lexparser/englishPCFG.ser.gz"
	args['debug']            = False
	args['num_instructions'] = 11
	
	# setup directions conversation
	setup(args)

	# run some tests
	run_test_samples()

