import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import csv
import math
import cmath
import copy

#returns heading of line between two points in radians
def heading(x1, y1, x2, y2):
	#slope = (y2-y1)/(x2-x1)
	#ret= np.arctan(slope)
	ret= math.atan2(y2 - y1, x2 - x1)
	return ret
	

#finds a new pont along a given heading at a given distance
def next_point(x0, y0, heading, distance):	
	x= x0 + distance*math.cos(heading) #distance units away
	y= y0 + distance*math.sin(heading) 
	new_point=[x,y]
	return new_point

#Mathematical function that calculates Horizontal Distance between two points
def distance(x1, y1, x2, y2):
	dist=math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
	return dist

#gets state of aircraft from list X in time t
def getState( F, t):
	indx = None 
	for i in F:
		if i[3]<=t:
			indx=F.index(i)
	if indx == len(F)-1:
		x_t = F[indx][0]
		y_t = F[indx][1]
		v_t = F[indx][2]
		h_t = 0.0
		state_t = {'x': x_t, 'y': y_t, 'v': v_t, 't': t, 'h': h_t} #[x,y,v,t,heading]
		return state_t			
	headng = heading(F[indx][0],F[indx][1], F[indx+1][0], F[indx+1][1])
	distnce = (t-F[indx][3])*(F[indx][2]*1.68/100)
	x_y_t = next_point(F[indx][0],F[indx][1], headng, distnce)
	x_t = x_y_t[0]
	y_t = x_y_t[1]
	v_t = F[indx][2]
	h_t = headng
	state_t = {'x': x_t, 'y': y_t, 'v': v_t, 't': t, 'h': h_t} #[x,y,v,t,heading]
	return state_t

#creates Data[]
def create_Data(FA, FB):
	#create a list of valid obvservable times
	T_FA=[]
	T_FB=[]

	for i in FA:
		T_FA.append(i[3])

	for i in FB:
		T_FB.append(i[3])

	T_min=max(T_FA[0],T_FB[0])
	T_max=min(T_FA[len(T_FA)-1],T_FB[len(T_FB)-1])

	T_FA = [i for i in T_FA if (i >= T_min and i <= T_max)]
	T_FB = [i for i in T_FB if (i >= T_min and i <= T_max)]
	Times=list(set(T_FA+T_FB))
	Times.sort()

	#get state of each plane for each element of Times
	# List = [dict = {time: , A: :{x: ,y: ,v: ,t: ,heading: }, B:{x: ,y: ,v: ,t: ,heading: }, ... ]
	Data = []
	for i in Times:
		state_A=getState(FA, i)
		state_B=getState(FB, i)
		dictionary = {'time': i, 'A': state_A, 'B': state_B}
		Data.append(dictionary)

	return Data

#function finds root if exists for the time equation
def find_root(s_x,s_y,v_x,v_y,D):
	a = (v_x*v_x)+(v_y*v_y)
	b = 2 * ((s_x*v_x)+(s_y*v_y))
	c = (s_x*s_x)+(s_y*s_y)-(D*D)

	# calculate the discriminant
	d = (b**2) - (4*a*c)

	if d<0: #no real root
		return [False]

	# find two solutions
	sol1 = (-b-cmath.sqrt(d))/(2*a)
	sol2 = (-b+cmath.sqrt(d))/(2*a)   

	return [True, sol1.real, sol2.real]

#to check conflict given two elements of Data
#Might be glitchy
def conflict_checker(D1, D2): 
	D = 6100/100 #horiz threshold
	t1 = D1['time']
	A_t1 = D1['A']
	B_t1 = D1['B']
	t2 = D2['time']

	head_A = A_t1['h']
	sx_A = A_t1['x']
	sy_A = A_t1['y']
	v_A  = A_t1['v'] * 1.68 / 100
	vx_A = v_A * math.cos(head_A)
	vy_A = v_A * math.sin(head_A)

	head_B = B_t1['h']
	sx_B = B_t1['x']
	sy_B = B_t1['y']
	v_B  = B_t1['v'] * 1.68 / 100
	vx_B = v_B * math.cos(head_B)
	vy_B = v_B * math.sin(head_B)

	s_x = sx_A-sx_B
	s_y = sy_A-sy_B 
	v_x = vx_A-vx_B
	v_y = vy_A-vy_B

	dot = (s_x*v_x)+(s_y*v_y)

	if dot > 0:
		return [False, 0.0]

	roots = find_root(s_x,s_y,v_x,v_y,D)
	if (roots[0]==False):
		return [False, 0.0]
 
	T_D = min(roots[1],roots[2])
	T_e = max(roots[1],roots[2])

	if (T_D>=0 or T_e>=0):
		#need to check if within t2 here!!!
		if (t1+T_D <= t2): 
			return [True, t1+T_D]
		else:
			return [False, 0.0]
	else:
		return [False, 0.0]

#def returns first conflict if exists
def conflict(FA, FB): # issue here
	is_conflict = False
	conflict_at = None
	#--------- make a list of times and state of each plane for each time.
	Data = create_Data (FA , FB) #mutable , so cannot be iterated over
	#detect if there is a conflict:
	for i in range(0,len(Data)-1):
		result = conflict_checker(Data[i],Data[i+1]) 
		if result[0]:
			is_conflict = True
			conflict_at = result[1]
			break #break when first conflict is detected
	return [is_conflict, conflict_at]

#function used to print board
def printSolutionWithLabels(board, FA, vA_list):
	num_FA_segments = len(FA)-1
	num_speeds = len(vA_list)
	print("---------------------------------") 
	columnsIndices = vA_list
	rowsIndices = [1, 2, 3, 4]
	print(" ", end="")
	for c in columnsIndices :
	    print(" " + str(c), end="")      #str() is useless, but it's good practice
	print()                              #linefeed
	for i in range(num_FA_segments):
		print(i+1, end="|  ") 
		for j in range(num_speeds):
			x = ''
			if  board[i][j] == 1:
				x = x + 'X'
			else:
				x = x + '-'
			print(x, end="   ") 
		print()

#function that creates a final solution from board
def FINAL(board, FA):
	FA_final = copy.deepcopy(FA)
	#creating the solution FA_bar from board
	for i in range(0, len(FA_final)-1):
		index = board[i].index(1)
		FA_final[i][2] = vA_list[index]
	for j in range(1,len(FA_final)):
		dist = distance(FA_final[j][0], FA_final[j][1], FA_final[j-1][0], FA_final[j-1][1])
		time = dist/(FA_final[j-1][2]*1.68/100)
		FA_final[j][3] = FA_final[j-1][3]+time
	return FA_final

#function that creates a temporary F
def ROWPLAN(temp_board, row, FA1, vA_list):
	FA2 = copy.deepcopy(FA1)
	for i in range(0, row+1):
		index = temp_board[i].index(1)
		FA2[i][2] = vA_list[index]
		del FA2[row+2:] #trim upto segment

	#assign new times from i+1 to last
	for j in range(1,len(FA2)):
		dist = distance(FA2[j][0], FA2[j][1], FA2[j-1][0], FA2[j-1][1])
		time = dist/(FA2[j-1][2]*1.68/100)
		FA2[j][3] = FA2[j-1][3]+time
	print("intermediate : ", FA2)
	return FA2

#function that check for conflict in f_list
def COMPATIBLE(FA2, F_list):
	is_conflict = False
	for F in F_list:
#		print("FA2: ",FA2)
#		print("F: ",F)
		result = conflict(FA2, F) ####IF ISSUE, LOOK for error IN CONFLICT() FIRST
		if result[0]:
			is_conflict = True
			break
	if is_conflict == False:
		return True
	else:
		return False


#function to check if solution upto row is conflict free in 
#the next interval that starts with row[time]
def SAFE(board, row, FA, F_list, vA_list):
	FA1 = copy.deepcopy(FA)
	#create temp_board
	temp_board = copy.deepcopy(board)
	#create a new Flightplan corresponding to temp_board
	FA2 = ROWPLAN(temp_board, row, FA1, vA_list)	
	compatible = COMPATIBLE(FA2, F_list)
	return compatible


def SOLVE(board, counter, num_FA_segments, num_speeds, FA, F_list, vA_list):
	if counter == 0:
		if SAFE(board, num_FA_segments-1, FA, F_list, vA_list):
			return True
	if counter > 0:
		exists = False
		FA = copy.deepcopy(FA)
		tem_board = copy.deepcopy(board)
		for current_v in range(0,num_speeds):
			print("current_v=",current_v)
			tem_board[num_FA_segments - counter][current_v] = 1
			printSolutionWithLabels(tem_board, FA, vA_list)
			safety = SAFE(tem_board, num_FA_segments - counter, FA, F_list, vA_list)
			assin = SOLVE(tem_board, counter - 1,  num_FA_segments, num_speeds, FA, F_list, vA_list)
			if safety == True and assin == True:
					board[num_FA_segments - counter][current_v] = 1
					return SOLVE(board, counter - 1,  num_FA_segments, num_speeds, FA, F_list, vA_list )
			else:
				tem_board[num_FA_segments - counter][current_v] = 0
				continue
			if safety == False:
				print("--------------------------SAFETY FAILED")
				printSolutionWithLabels(tem_board, FA, vA_list)
				print("--------------------------SAFETY FAILED")
				tem_board[num_FA_segments - counter][current_v] = 0
				continue
		if exists == False:
			return False


#******************* Getting DUMMMY FLIGHTPLANS *******************
print("(*******Scale : 1 unit = 100 feet*******)\n\n")
#importing the flighplans
import flightplans as fp
FA = fp.FA
FB = fp.FB
FC = fp.FC
FD = fp.FD
vA_list = fp.vA_list
vA_list.sort(reverse=True)

print("Original FA: ",FA)
print("FB: ",FB)
#print("FC: ",FC)
#sWe will vary FA in this code
F_list = [FB, FC, FD] #create F_list
#create the matrix
num_FA_segments = len(FA)-1
num_speeds = len(vA_list)
board = [[0 for x in range(num_speeds)] for y in range(num_FA_segments)]
print("INITIAL EMPTY BOARD:")
printSolutionWithLabels(board, FA, vA_list)


#finding a solution
is_solved=SOLVE(board, num_FA_segments, num_FA_segments, num_speeds, FA, F_list, vA_list)

if is_solved:
	print()
	print("a solution has been found!!")
	print("FINAL BOARD:")
	printSolutionWithLabels(board, FA, vA_list)	#assign new airspeeds to FA
	#generate new FA from solution board
	FA_new = FINAL(board, FA)
	#final check for compatibility
	print("\nSolution FA: ",FA_new)
	print("\nFinal check:--")
	if COMPATIBLE(FA_new, F_list):
		print ("\nF_list is now compatible with NEW FA!!")
else:
	print("No solution could be found")