#!/usr/bin/env python

##############################################
# import libraries
##############################################
import copy
import nltk
import unicodedata
from nltk.tree import *

##############################################
# given a tree, extract out the words
##############################################
def get_tree_words(tree):
	# base case: leaf node
	if isinstance(tree[0], unicode):
		return get_word(tree)
	# inductive case: subtrees
	result = ''
	for subtree in tree:
		if isinstance(subtree, ParentedTree):
			result += get_tree_words(subtree) + ' '
	return ' '.join(result.split())

##############################################
# given a tree, prune all subtrees that are in the list_of_tags
##############################################
def prune_tree(tree, list_of_tags):
	temp_tree = tree.copy(deep=True) 
	idxs_to_delete = []
	for idx, subtree in enumerate(temp_tree):
		if isinstance(subtree, ParentedTree) and subtree.label() in list_of_tags:
			idxs_to_delete.append(idx)
	for idx in range(len(temp_tree)-1,-1,-1):
		if idx in idxs_to_delete:			
			del temp_tree[idx]
	for idx, subtree in enumerate(temp_tree):
		if isinstance(subtree, ParentedTree):
			temp_tree[idx] = prune_tree(subtree, list_of_tags)
	return temp_tree
	
##############################################
# find a leaf node that contains word
# returns False if no leafs have that word
# returns Tree whose value matches the word
##############################################
def get_leaf_node(tree, word):
	# base case: found word
	if tree[0] == word:
		return [True, tree]
	# look among subtrees for this word
	for subtree in tree:
		if isinstance(subtree, ParentedTree):
			result = get_leaf_node(subtree, word)
			if result[0]:
				return result
	# it was not found, return failure	
	return [False, None]

##############################################
# look for nearest parent that with specified label
##############################################
def get_parent_tree_with_tag(tree, tag):
	# base case: found parent with tag
	if isinstance(tree, ParentedTree) and tree.label() == tag:
		return tree
	# if this tree has a parent, check the parent
	if isinstance(tree, ParentedTree) and not tree.parent() == None:
		result = get_parent_tree_with_tag(tree.parent(), tag)
		if type(result) == ParentedTree:
			return result
	# it was not found, return failure	
	return False

##############################################
# get the leaf node to the right
##############################################
def get_next_leaf_node(leaf_node):
	return get_next_leaf_node_helper(leaf_node.parent(), leaf_node.parent_index()+1)
def get_next_leaf_node_helper(tree, start_child_idx):
	# base case: leaf node
	if isinstance(tree[0], unicode):
		return tree
	# loop over children (starting at child index passed in)
	# inductive case: this node has children; start with the left-most child, then work rightwards 
	for idx in range(start_child_idx, len(tree)):
		if isinstance(tree[idx], ParentedTree):
			result = get_next_leaf_node_helper(tree[idx], 0)
			if isinstance(result, ParentedTree):
				return result
	# inductive case: this node has a parent
	if isinstance(tree, ParentedTree) and isinstance(tree.parent(), ParentedTree):
		# discover which child idx you are of your parent
		parent_index = tree.parent_index()
		result = get_next_leaf_node_helper(tree.parent(), parent_index+1)
		if isinstance(result, ParentedTree):
			return result
	# never found such a leaf, return failure
	return None	

##############################################
# get the leaf node to the left
##############################################
def get_prev_leaf_node(tree):
	return get_prev_leaf_node_helper(tree.parent(), tree.parent_index()-1)
def get_prev_leaf_node_helper(tree, start_child_idx):
	# base case: leaf node
	if isinstance(tree[0], unicode):
		return tree
	# loop over children (starting at child index passed in)
	# inductive case: this node has children; start with the right-most child, then work leftwards 
	for idx in range(start_child_idx, -1, -1):
		if isinstance(tree[idx], ParentedTree):
			result = get_prev_leaf_node_helper(tree[idx], 0)
			if isinstance(result, ParentedTree):
				return result
	# inductive case: this node has a parent
	if isinstance(tree, ParentedTree) and isinstance(tree.parent(), ParentedTree):
		# discover which child idx you are of your parent
		parent_index = tree.parent_index()
		result = get_prev_leaf_node_helper(tree.parent(), parent_index-1)
		if isinstance(result, ParentedTree):
			return result
	# never found such a leaf, return failure
	return None	

##############################################
# search a list of leafs (which are ordered left to right) 
# provides special allowance: list_of_words can contain 2-word strings
#
# return all the leafs whose words come from a list of words
# return results = [['word', leaf_node, idx], ['word', leaf_node, idx], ...]
##############################################
def get_all_leaf_nodes_with_word(list_of_leaf_nodes, list_of_words):
	results = []

	for idx in range(len(list_of_leaf_nodes)):
		word = get_word(list_of_leaf_nodes[idx])
		# since some items in the list are two_words, we get the node to the right and check it too
		two_words = ''
		if idx < len(list_of_leaf_nodes)-1:
			two_words = word + ' ' + get_word(list_of_leaf_nodes[idx+1])
		# check if either word is in our list_of words
		if word in list_of_words:
			results.append([word, list_of_leaf_nodes[idx], idx])
		elif two_words in list_of_words:
			results.append([two_words, list_of_leaf_nodes[idx], idx])

	return results

##############################################
# find the earliest leaf node (leftwards) from the given POS tags
##############################################
def get_earliest_prev_leaf_with_pos(tree, pos_tags):
	# get the prev leaf node
	prev_leaf_node = tree

	# loop until you get to the front of the tree
	while isinstance(prev_leaf_node, ParentedTree):
		# this is a leaf node of the desired pos
		if get_label(prev_leaf_node) in pos_tags:
			return prev_leaf_node
		else:
			prev_leaf_node = get_prev_leaf_node(prev_leaf_node)
	# never found such a leaf, return failure
	return None	

##############################################
# find the nearest parent with with the given POS tag
##############################################
def get_nearest_parent_of_pos(tree, pos_tags):
	# base case: this is a parent node of the desired pos
	if isinstance(tree, ParentedTree) and get_label(tree) in pos_tags:
		return tree
	# inductive case: this node has a parent
	if isinstance(tree, ParentedTree) and isinstance(tree.parent(), ParentedTree):
		result = get_nearest_parent_of_pos(tree.parent(), pos_tags)
		if isinstance(result, ParentedTree):
			return result
	# never found such a parent, return failure
	return None	

##############################################
# find the nearest ancestor of two tree nodes
##############################################
def get_nearest_ancestor(orig_tree1, tree1, tree2):
	# base case: tree1 and tree2 are the same tree
	if isinstance(tree1, ParentedTree) and tree1 == tree2:
		return tree1
	# inductive case: check tree2 against all of tree1's parents
	if isinstance(tree1, ParentedTree) and isinstance(tree1.parent(), ParentedTree):
		result = get_nearest_ancestor(orig_tree1, tree1.parent(), tree2)
		if isinstance(result, ParentedTree):
			return result
	else:
		return None
	return get_nearest_ancestor(orig_tree1, orig_tree1, tree2.parent())

##############################################
# search a tree and return all the leafs
# leaf_nodes = [leaf_node1, leaf_node2, ...]
##############################################
def flatten_tree(tree):
	# base case: leaf node
	if isinstance(tree[0], unicode):
		return [tree]
	# inductive case: subtrees
	leafs = []
	for subtree in tree:
		if isinstance(subtree, ParentedTree):
			result = flatten_tree(subtree)
			if len(result) > 0 and isinstance(result[0], list):
				leafs = leafs + result
			elif not result == []:
				leafs = leafs + result
	return leafs

##############################################
# get all leaf nodes between two indeces (inclusive)
##############################################
def get_leaf_nodes_between_two_indeces(tree, idx1, idx2):
	leafs = flatten_tree(tree)
	result = []
	for idx in range(idx1, idx2+1):
		result.append(leafs[idx])
	return result

##############################################
# get the sequence of words from a list of leafs 
##############################################
def get_words_from_list_of_leaf_nodes(leaf_nodes):
	words = ''
	for leaf_node in leaf_nodes:
		words += get_word(leaf_node) + ' '
	return words

##############################################
# get the label of a tree node
##############################################
def get_label(tree):
	return unicodedata.normalize('NFKD', tree.label()).encode('ascii','ignore')

##############################################
# get the word of a leaf node
##############################################
def get_word(tree):
	return unicodedata.normalize('NFKD', tree[0]).encode('ascii','ignore')

##############################################
# get the idx of the leaf in a tree 
##############################################
def get_leaf_idx(tree, leaf, start_idx, direction):
	leaf_list = flatten_tree(tree)

	if direction == 'right':
		for idx in range(start_idx, len(leaf_list)):
			if leaf_list[idx] == leaf:
				return idx
	elif direction == 'left':
		for idx in range(start_idx, -1, -1):
			if leaf_list[idx] == leaf:
				return idx
	else:
		print 'bad direction passed in!'
	return None

