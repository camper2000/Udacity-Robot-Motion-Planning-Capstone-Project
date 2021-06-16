import numpy as np
import copy
import pandas as pd
import random

# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        
        # goal destination area
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        
        # total step number; whether hit goal; round 0 or 1
        self.steps = 0
        self.hitgoal = 0
        self.round = 0
        
        # visit time of each cell
        self.visits = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.visits[maze_dim-1][0] = 100
        
        # whether a cell is a dead end or leads to a dead end, 1 is dead, 2 will lead to a dead end ,0 is not
        self.deads = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.deads[maze_dim-1][0] = 1
        self.deads[maze_dim-2][0] = 2
        
        # list of vertical walls, 0 is wall; 1 is no wall, can pass through
        self.wallv = [[0 for i in range(self.maze_dim-1)] for j in range(self.maze_dim)]
        # list of horizontal walls
        self.wallh = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim-1)]
        
        # indicate A* G value of each node
        self.G = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        # whether A* G value is updated. If yes, its neighbors' G value may be also updated, whose neighbors will have a chain effect
        self.G_updated = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        
        # the A* open and close list of node
        self.close_list = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.open_list = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        
        # indicate the parent node of a node
        self.parents = [[[-1, -1] for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        
        # destination point
        self.x_end = -1
        self.y_end = -1
        # for calculate A* H value, which is more accurate than (x_end, y_end)
        self.x_end_0 = -1
        self.y_end_0 = -1
        # path from start point to the destination
        self.path = []
        
        #lists to record A* Search cost and gain (path length)
        self.astarcost = [0]
        self.path_len = []
        
        # path to execute when we need the robot to go to a specific location,
        self.pathexe = []
        # whether to continue previous unfinished execution path
        self.contd = 0
        
        #for test
        self.step1 = 0
        #test 0:no randomness; 1:less randomness; 2:more randomness
        self.randomness = 0
        
    # A* H value of given (x,y) before hitting goal
    def get_H1(self, x, y):
        return min(abs(self.goal_bounds[0] - x), abs(self.goal_bounds[1] - x)) + min(abs(self.goal_bounds[0] - y), abs(self.goal_bounds[1] - y))
    
    # updated A* H value of given (x,y) after hitting goal (destination point first time reached)
    def get_H2(self, x, y):
        path_steps = len(self.path)
        return (abs(self.x_end_0 - x) + abs(self.y_end_0 - y)) / (self.x_end_0 + self.y_end_0) * path_steps
    
    # check whether hit goal area
    def check_hitgoal(self, location):
        if location[0] in self.goal_bounds and location[1] in self.goal_bounds:
            return True
        else:
            return False
        
    def get_parent(self, node):
        return self.parents[self.maze_dim-node[1]-1][node[0]]
    
    # check whether robot can go to the destination node directly, if yes, return rotation and movement
    def check_pass(self, dest):
        x = self.location[0]
        y = self.location[1]
        dx = dest[0] - x
        dy = dest[1] - y
        # same point
        if dx ==0 and dy == 0:
            return 0, 0
        # impossible to go to destination node directly
        elif dx != 0 and dy != 0:
            return -1, -1
        else:
            rotation = 0
            movement = 0
            distance = max(abs(dx),abs(dy))
            if distance > 3:
                return -2, -2
            else:
                direction = [dx/distance, dy/distance]
                dir_dest = list(dir_move.keys())[list(dir_move.values()).index(direction)]
                dir_change_list = dir_sensors.get(self.heading)
                for move in range(1, distance + 1):
                    #horizontal
                    if direction[0] == 0:
                        x_h = int(x)
                        y_h = int(y - 0.5 + (move - 0.5) * direction[1])
                        if y_h >= 0 and y_h <= self.maze_dim - 2:
                            if self.wallh[self.maze_dim-y_h-2][x_h] == 1:
                                movement = move
                            else:
                                break
                        else:
                            return -3, -3
                    #vertical
                    elif direction[1] == 0:
                        x_v = int(x - 0.5 + (move - 0.5) * direction[0])
                        y_v = int(y)
                        if x_v >= 0 and x_v <= self.maze_dim - 2:
                            if self.wallv[self.maze_dim-y_v-2][x_v] == 1:
                                movement = move
                            else:
                                break
                        else:
                            return -4, -4
                if dir_dest in dir_change_list:
                    dir_change = dir_change_list.index(dir_dest)
                    if dir_change == 0:
                        rotation = -90
                    elif dir_change == 2:
                        rotation = 90
                else:
                    movement = 0-movement
        movement = int(movement)
        return rotation, movement
    
    # the number of open squares in back direction
    def check_back(self):
        x = self.location[0]
        y = self.location[1]
        direction = dir_move[self.heading]
        sensor_back = 0
        for i in range(1, 4):
            #horizontal
            if direction[0] == 0:
                x_h = int(x)
                y_h = int(y - 0.5 - (i - 0.5) * direction[1])
                if y_h >= 0 and y_h <= self.maze_dim - 2:
                    if self.wallh[self.maze_dim-y_h-2][x_h] == 1:
                        sensor_back = i
                    else:
                        break
            #vertical
            elif direction[1] == 0:
                x_v = int(x - 0.5 - (i - 0.5) * direction[0])
                y_v = int(y)
                if x_v >= 0 and x_v <= self.maze_dim -2:
                    if self.wallv[self.maze_dim-y_v-2][x_v] == 1:
                        sensor_back = i
                    else:
                        break
        return sensor_back
     
    # update neighbor walls of horizontal and vertical accroding to sensor
    def update_neighwall(self, sensors):
        x = self.location[0]
        y = self.location[1]
        for i in range(3):
            if sensors[i] > 0:
                direction = self.heading
                if i == 0:
                    direction = dir_sensors[self.heading][0]
                if i == 1:
                    direction = dir_sensors[self.heading][1]
                elif i == 2:
                    direction = dir_sensors[self.heading][2]
                direction = dir_move[direction]
                for move in range(1, sensors[i] + 1):
                    #horizontal
                    if direction[0] == 0:
                        x_h = int(x)
                        y_h = int(y - 0.5 + (move - 0.5) * direction[1])
                        if y_h >= 0 and y_h <= self.maze_dim -2:
                            self.wallh[self.maze_dim-y_h-2][x_h] = 1
                    #vertical
                    elif direction[1] == 0:
                        x_v = int(x - 0.5 + (move - 0.5) * direction[0])
                        y_v = int(y)
                        if x_v >= 0 and x_v <= self.maze_dim - 2:
                             self.wallv[self.maze_dim-y_v-2][x_v] = 1
        
    def exe_path(self, sensors):
        next_pos = self.pathexe[0]
        self.pathexe.pop(0)
        if len(self.pathexe) == 0:
            self.contd = 0
        rotation, movement = self.check_pass(next_pos)
        
        # update parameters
        if rotation == -90:
            self.heading = dir_sensors[self.heading][0]
        elif rotation == 90:
            self.heading = dir_sensors[self.heading][2]
        self.location[0] += dir_move[self.heading][0] * movement
        self.location[1] += dir_move[self.heading][1] * movement
        if not self.check_hitgoal(self.location):
            self.steps += 1
        
        return rotation, movement
    
    def update_path(self):
        self.path = []
        curr_loc = [self.x_end, self.y_end]
        while curr_loc[0] != 0 or curr_loc[1] != 0:
            self.path.append(list(curr_loc))
            curr_loc = self.get_parent(curr_loc)
        self.path.append([0,0])
        self.path.reverse()

    def update_visited(self):
        i_update = 1
        while i_update == 1:
            i_update == 0
            df = pd.DataFrame(columns=['x', 'y', 'F_value', 'visit', 'G_updated'])
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    visit = self.visits[self.maze_dim-j-1][i]
                    if self.open_list[self.maze_dim-j-1][i] == 1 and visit > 0:
                        if (not self.check_hitgoal([i,j])) or (i == self.x_end and j == self.y_end):
                            G_updated = self.G_updated[self.maze_dim-j-1][i]
                            G_value = self.G[self.maze_dim-j-1][i]
                            F_value = G_value + self.get_H2(i,j)
                            df = df.append({'x':i, 'y':j, 'F_value':F_value, 'visit': visit, 'G_updated': G_updated}, ignore_index=True)
            if df.shape[0] > 0:
                i_update = 1
                df = df.sort_values(['F_value', 'G_updated', 'visit'], ascending=[True, False, True], ignore_index=True)
                x = int(df.reset_index()['x'].iloc[0])
                y = int(df.reset_index()['y'].iloc[0])
                
                for direction in [[0, 1], [1, 0], [0, -1], [-1, 0]]:
                    for move in range (1, 4):
                        x_new = x + move * direction[0]
                        y_new = y + move * direction[1]
                        if x_new >= 0 and x_new <= self.maze_dim - 1 and y_new >= 0 and y_new <= self.maze_dim - 1:
                            #check horizontal wall
                            if direction[0] == 0:
                                x_h = int(x)
                                y_h = int(y - 0.5 + (move - 0.5) * direction[1])
                                if y_h >= 0 and y_h <= self.maze_dim - 2:
                                    if self.wallh[self.maze_dim-y_h-2][x_h] == 1:
                                        if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                                            Original_G = self.G[self.maze_dim-y_new-1][x_new]
                                            new_G = self.G[self.maze_dim-y-1][x] + 1
                                            # if exists in close list
                                            if self.close_list[self.maze_dim-y_new-1][x_new] == 1:
                                                if Original_G > new_G:
                                                    self.close_list[self.maze_dim-y_new-1][x_new] = 0
                                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                            else:
                                                # if not exists in open list, add it to open list
                                                if self.open_list[self.maze_dim-y_new-1][x_new] == 0:
                                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                # if exists in open list and the G value in open list is larger, then update its value and parent
                                                elif Original_G > new_G:
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                        else:
                                            break
                                    else:
                                        break
                                else:
                                    break
                            #check vertical wall
                            elif direction[1] == 0:
                                x_v = int(x - 0.5 + (move - 0.5) * direction[0])
                                y_v = int(y)
                                if x_v >= 0 and x_v <= self.maze_dim - 2:
                                    if self.wallv[self.maze_dim-y_v-2][x_v] == 1:
                                        if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                                            Original_G = self.G[self.maze_dim-y_new-1][x_new]
                                            new_G = self.G[self.maze_dim-y-1][x] + 1
                                            # if exists in close list
                                            if self.close_list[self.maze_dim-y_new-1][x_new] == 1:
                                                if Original_G > new_G:
                                                    self.close_list[self.maze_dim-y_new-1][x_new] = 0
                                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                            else:
                                                # if not exists in open list, add it to open list
                                                if self.open_list[self.maze_dim-y_new-1][x_new] == 0:
                                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                # if exists in open list and the G value in open list is larger, then update its value and parent
                                                elif Original_G > new_G:
                                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                        else:
                                            break
                                    else:
                                        break
                                else:
                                    break
                        else:
                            break
                self.open_list[self.maze_dim-y-1][x] = 0
                self.close_list[self.maze_dim-y-1][x] = 1   
                self.G_updated[self.maze_dim-y-1][x] = 0
                self.visits[self.maze_dim-self.location[1]-1][self.location[0]] += 1
                self.update_path()
            else:
                i_update = 0
        
    # finish 1st round, start 2nd round, reset paremeters
    def reset_second(self):
        #for test
        self.steps += 1
        print("1st time hit goal steps, run 0 total steps: {}, {}".format(self.step1, self.steps))
        print("path: {}".format(self.path))
        print("length: {}".format(len(self.path)-1))
        # print(self.open_list)
        # print(self.close_list)
        # print(self.parents)
        # print(self.visits)
        
        self.location = [0, 0]
        self.heading = 'up'
        self.contd = 0
        self.steps = 0
        self.hitgoal = 0
        self.round = 1
            
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        rotation = 0
        movement = 0
                
        # update hitgoal when hits goal in the 1st round
        if self.round == 0 and self.hitgoal == 0 and self.check_hitgoal(self.location):
            self.hitgoal = 1
            # Mark end point in goal area. Since there are 4 nodes in goal area, we pick the one most near the goal area entrance 
            x = self.location[0]
            y = self.location[1]
            parent = self.get_parent(self.location)
            xp = parent[0]
            yp = parent[1]
            dx = x - xp
            dy = y - yp
            move = max(abs(dx),abs(dy))
            # check one step backwards
            x_back = int(x - dx/move)
            y_back = int(y - dy/move)
            if self.check_hitgoal([x_back, y_back]):
                self.x_end = x_back
                self.y_end = y_back
                self.x_end_0 = x_back - dx/move
                self.y_end_0 = y_back - dy/move
                self.parents[self.maze_dim-y_back-1][x_back][0] = xp
                self.parents[self.maze_dim-y_back-1][x_back][1] = yp
            else:
                self.x_end = x
                self.y_end = y
                self.x_end_0 = x_back
                self.y_end_0 = y_back
            
            #for test
            self.step1 = self.steps
        
        #update walls
        if self.round == 0:
            self.update_neighwall(sensors)
              
        # stop 1st round and start 2nd round
        # when: 1. total steps > 900 or 2. total hitgoal times > 3 or 3. total explore open list times; 4. total visit
        if self.round == 0 and self.hitgoal == 1 and self.steps > 900:
            self.reset_second()
            return ('Reset', 'Reset')
        
        x = self.location[0]
        y = self.location[1]
        
        # mark if dead ends
        if sensors == [0,0,0]:
            self.deads[self.maze_dim-y-1][x] = 1
        # if the node in front is a dead end or lead to dead end, mark current position lead to dead end
        elif sensors[0] == 0 and sensors[2] == 0:
            x_front_one = x + dir_move[self.heading][0]
            y_front_one = y + dir_move[self.heading][1]
            if self.deads[self.maze_dim-y_front_one-1][x_front_one] > 0:
                self.deads[self.maze_dim-y-1][x] = 2
         
        # execute planned path
        if self.contd == 1:
            return self.exe_path(sensors)
        else:
            if self.round == 0:
            
                self.G_updated[self.maze_dim-y-1][x] = 0
                
                # not hit goal, focus more on hitting goal
                if self.hitgoal == 0:
                
                    # dataframe for all the neighbours
                    df = pd.DataFrame(columns=['x_new', 'y_new', 'direction', 'forward', 'moves', 'abs_moves', 'F_value', 'visit', 'G_updated'])
                    # check backward neighbours
                    sensor_back = self.check_back()
                    for move in range(0-sensor_back, 0, 1):
                        x_new = x + dir_move[self.heading][0] * move
                        y_new = y + dir_move[self.heading][1] * move
                        if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                            df = df.append({'x_new':x_new, 'y_new':y_new, 'direction': 1, 'forward': -1, 'moves': move, 'abs_moves': abs(move), 'G_updated': 0}, ignore_index=True)
                    # check forward neighbours 
                    for i in range(3):
                        if sensors[i] > 0:
                            heading_new = self.heading
                            if i == 0:
                                heading_new = dir_sensors[self.heading][0]
                            elif i == 2:
                                heading_new = dir_sensors[self.heading][2]
                            for move in range(1,min(sensors[i],3)+1):
                                x_new = x + dir_move[heading_new][0] * move
                                y_new = y + dir_move[heading_new][1] * move
                                if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                                    df = df.append({'x_new':x_new, 'y_new':y_new,'direction': i, 'forward':i%2, 'moves': move, 'abs_moves': abs(move),'G_updated': 0}, ignore_index=True)
                    #loop through df, update open list, close list, parent, A* G value and other information of each neighbour
                    for index, row in df.iterrows():
                        x_new = int(row['x_new'])
                        y_new = int(row['y_new'])
                        Original_G = self.G[self.maze_dim-y_new-1][x_new]
                        new_G = self.G[self.maze_dim-y-1][x] + 1
                        # if exists in close list                        
                        if self.close_list[self.maze_dim-y_new-1][x_new] == 1:
                            if Original_G > new_G:
                                self.close_list[self.maze_dim-y_new-1][x_new] = 0
                                self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                self.G[self.maze_dim-y_new-1][x_new] = new_G
                                self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                F_value = self.G[self.maze_dim-y_new-1][x_new] + self.get_H1(x_new,y_new)
                                self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                row['F_value'] = F_value
                                row['visit'] = self.visits[self.maze_dim-y_new-1][x_new]
                                row['G_updated'] = 1
                            else:
                                row['F_value'] = 99999
                                row['visit'] = self.visits[self.maze_dim-y_new-1][x_new]
                        else:
                            # if not exists in open list, add it to open list
                            if self.open_list[self.maze_dim-y_new-1][x_new] == 0:
                                self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                self.G[self.maze_dim-y_new-1][x_new] = new_G
                                self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                self.parents[self.maze_dim-y_new-1][x_new][1] = y
                            # if exists in open list and the G value in open list is larger, then update its value and parent
                            elif Original_G > new_G:
                                self.G[self.maze_dim-y_new-1][x_new] = new_G
                                self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                row['G_updated'] = 1
                            F_value = self.G[self.maze_dim-y_new-1][x_new] + self.get_H1(x_new,y_new)
                            row['F_value'] = F_value
                            row['visit'] = self.visits[self.maze_dim-y_new-1][x_new]
                            
                    # move the current node from open to close list
                    self.open_list[self.maze_dim-y-1][x] = 0
                    self.close_list[self.maze_dim-y-1][x] = 1

                    # priority to choose to which neighboor to move:
                    # 1. small F_value; 2. less visit time; 3. moving forward (1) comes first, backwards (-1) last; 4. large movement
                    df = df.sort_values(['F_value', 'visit', 'G_updated', 'forward', 'abs_moves'], ascending=[True, True, False, False, False], ignore_index=True) 
                    rotation = 0
                    #no randomness
                    row_num = 0
                    
                    #test
                    if self.randomness > 0:
                        # add some randomness so that the robot can explore the whole map better                     
                        if self.steps < self.maze_dim * 25:
                            row_count = df.shape[0]
                            numberList = []
                            weightList = []
                            for i in range(row_count):
                                numberList.append(i)
                                weightList.append((row_count-i)**(3-self.randomness))
                            row_num = random.choices(numberList, weights=weightList, k=1)[0]
                    
                    # check if can hit goal directly
                    for index, row in df.iterrows():
                        x_new = int(row['x_new'])
                        y_new = int(row['y_new'])
                        if self.check_hitgoal([x_new, y_new]):
                            row_num = index

                    direction_i = df.reset_index()['direction'].iloc[row_num]
                    if direction_i == 0:
                        rotation = -90
                    elif direction_i == 2:
                        rotation = 90
                    movement = int(df.reset_index()['moves'].iloc[row_num])   
                    
                    # update parameter
                    if rotation == -90:
                        self.heading = dir_sensors[self.heading][0]
                    elif rotation == 90:
                        self.heading = dir_sensors[self.heading][2]
                    self.location[0] += dir_move[self.heading][0] * movement
                    self.location[1] += dir_move[self.heading][1] * movement
                    self.location[0] = int(self.location[0])
                    self.location[1] = int(self.location[1])
                    self.visits[self.maze_dim-self.location[1]-1][self.location[0]] += 1
                    if not self.check_hitgoal(self.location):
                        self.steps += 1
                    
                    return rotation, movement
            
                # already hit goal, focus more on exploring the map, focus more on open list with the least F value
                else:
                    # update path visited
                    self.update_visited()
                    
                    df_neighbor = pd.DataFrame(columns=['x_new', 'y_new', 'direction', 'moves'])
                    #check backward neighbours
                    sensor_back = self.check_back()
                    for move in range(0-sensor_back, 0, 1):
                        x_new = x + dir_move[self.heading][0] * move
                        y_new = y + dir_move[self.heading][1] * move
                        df_neighbor = df_neighbor.append({'x_new':x_new, 'y_new':y_new, 'direction': 1, 'moves': move}, ignore_index=True)
                        if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                            Original_G = self.G[self.maze_dim-y_new-1][x_new]
                            new_G = self.G[self.maze_dim-y-1][x] + 1
                            # if exists in close list
                            if self.close_list[self.maze_dim-y_new-1][x_new] == 1:
                                if Original_G > new_G:
                                    self.close_list[self.maze_dim-y_new-1][x_new] = 0
                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                            else:
                                # if not exists in open list, add it to open list
                                if self.open_list[self.maze_dim-y_new-1][x_new] == 0:
                                    self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                # if exists in open list and the G value in open list is larger, then update its value and parent
                                elif Original_G > new_G:
                                    self.G[self.maze_dim-y_new-1][x_new] = new_G
                                    self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                    self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                    self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                    # check forward neighbours
                    for i in range(3):
                        if sensors[i] > 0:
                            heading_new = self.heading
                            if i == 0:
                                heading_new = dir_sensors[self.heading][0]
                            elif i == 2:
                                heading_new = dir_sensors[self.heading][2]
                            for move in range(1,min(sensors[i],3)+1):
                                x_new = x + dir_move[heading_new][0] * move
                                y_new = y + dir_move[heading_new][1] * move
                                df_neighbor = df_neighbor.append({'x_new':x_new, 'y_new':y_new, 'direction': i, 'moves': move}, ignore_index=True)
                                if self.deads[self.maze_dim-y_new-1][x_new] == 0:
                                    Original_G = self.G[self.maze_dim-y_new-1][x_new]
                                    new_G = self.G[self.maze_dim-y-1][x] + 1
                                    # if exists in close list
                                    if self.close_list[self.maze_dim-y_new-1][x_new] == 1:
                                        if Original_G > new_G:
                                            self.close_list[self.maze_dim-y_new-1][x_new] = 0
                                            self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                            self.G[self.maze_dim-y_new-1][x_new] = new_G
                                            self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                            self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                            self.G_updated[self.maze_dim-y_new-1][x_new] = 1
                                    else:
                                        # if not exists in open list, add it to open list
                                        if self.open_list[self.maze_dim-y_new-1][x_new] == 0:
                                            self.open_list[self.maze_dim-y_new-1][x_new] = 1
                                            self.G[self.maze_dim-y_new-1][x_new] = new_G
                                            self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                            self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                        # if exists in open list and the G value in open list is larger, then update its value and parent
                                        elif Original_G > new_G:
                                            self.G[self.maze_dim-y_new-1][x_new] = new_G
                                            self.parents[self.maze_dim-y_new-1][x_new][0] = x
                                            self.parents[self.maze_dim-y_new-1][x_new][1] = y
                                            self.G_updated[self.maze_dim-y_new-1][x_new] = 1

                    # move the current position from open to close list
                    self.open_list[self.maze_dim-y-1][x] = 0
                    self.close_list[self.maze_dim-y-1][x] = 1
                    
                    # go to node in open list with least A* F_value
                    df = pd.DataFrame(columns=['x_new', 'y_new', 'F_value', 'visit', 'G_updated', 'neighbor', 'G_neighbor', 'direction', 'moves'])
                    for i in range(self.maze_dim):
                        for j in range(self.maze_dim):
                            if self.open_list[self.maze_dim-j-1][i] == 1 and (i != self.location[0] or j != self.location[1]) and self.deads[self.maze_dim-j-1][i] == 0:
                                # include only one end node, not include other 3 goal nodes
                                if (not self.check_hitgoal([i,j])) or (i == self.x_end and j == self.y_end):
                                    G_updated = self.G_updated[self.maze_dim-j-1][i]
                                    visit = self.visits[self.maze_dim-j-1][i]
                                    G_value = self.G[self.maze_dim-j-1][i]
                                    F_value = G_value + self.get_H2(i,j)
                                    neighbor = 0
                                    direction = 0
                                    moves = 0
                                    for index, row in df_neighbor.iterrows():
                                        x_neighbor = int(row['x_new'])
                                        y_neighbor = int(row['y_new'])
                                        if i == x_neighbor and j == y_neighbor:
                                            neighbor = 1
                                            direction = int(row['direction'])
                                            moves = int(row['moves'])
                                            break
                                    df = df.append({'x_new':i, 'y_new':j, 'F_value':F_value, 'visit': visit, 'G_updated': G_updated, 'neighbor': neighbor, 'G_neighbor': int((G_updated+neighbor)/2), 'direction': direction, 'moves': moves}, ignore_index=True)
                    # update parameters
                    self.visits[self.maze_dim-self.location[1]-1][self.location[0]] += 1
                    if df.shape[0] > 0:
                        df = df.sort_values(['G_neighbor', 'visit', 'G_updated', 'neighbor', 'F_value'], ascending=[False, True, False, False, True], ignore_index=True)
                        x_new = int(df.reset_index()['x_new'].iloc[0])
                        y_new = int(df.reset_index()['y_new'].iloc[0])
                        neighbor = int(df.reset_index()['neighbor'].iloc[0])
                        rotation = 0
                        movement = 0
                        if neighbor == 1:
                            direction = int(df.reset_index()['direction'].iloc[0])
                            if direction == 0:
                                rotation = -90
                                self.heading = dir_sensors[self.heading][0]
                            elif direction == 2:
                                rotation = 90
                                self.heading = dir_sensors[self.heading][2]
                            movement = int(df.reset_index()['moves'].iloc[0])
                            self.location[0] += dir_move[self.heading][0] * movement
                            self.location[1] += dir_move[self.heading][1] * movement
                            self.location[0] = int(self.location[0])
                            self.location[1] = int(self.location[1])
                            if not self.check_hitgoal(self.location):
                                self.steps += 1
                            
                            return rotation, movement
                        else:
                            #if the total cost of recent A* Search is more than the gain (shorten path steps), proceed to 2nd run
                            astar_num = 6
                            self.path_len.insert(0,len(self.path))
                            if len(self.astarcost) > astar_num:
                                total_cost = 0
                                for i in range(0,astar_num - 1):
                                    total_cost += self.astarcost[i]
                                gain = (self.path_len[astar_num - 1] - self.path_len[0]) * 30
                                if total_cost > gain:
                                    self.reset_second()
                                    return ('Reset', 'Reset')
                            
                            #find a path from current location to the un-visited node with least F-value
                            path_dest = []
                            path_curr = []
                            parent_dest = [x_new, y_new]
                            parent_curr = self.location
                            while parent_dest not in self.path:
                                path_dest.insert(0, parent_dest)
                                parent_dest = self.get_parent(parent_dest)
                            while parent_curr not in self.path:
                                path_curr.insert(0, parent_curr)
                                parent_curr = self.get_parent(parent_curr)
                            index_pdest = self.path.index(parent_dest)
                            index_pcurr = self.path.index(parent_curr)
                            self.pathexe = []
                            if index_pdest > index_pcurr:
                                for i in range(index_pcurr,index_pdest+1):
                                    self.pathexe.append(self.path[i])
                            else:
                                for i in range(index_pdest,index_pcurr+1):
                                    self.pathexe.append(self.path[i])
                                self.pathexe.reverse()
                            for node in path_dest:
                                 self.pathexe.append(node)
                            for node in path_curr:
                                 self.pathexe.insert(0, node)
                            
                            self.pathexe.pop(0)
                            self.contd = 1
                            self.astarcost.insert(0, len(self.pathexe))
                            return self.exe_path(sensors)
                    else:
                        # update parameter
                        self.reset_second()                        
                        return ('Reset', 'Reset')
                    
            # 2nd round
            else:
                self.pathexe = []
                self.pathexe = self.path
                self.pathexe.pop(0)
                self.contd = 1
                return self.exe_path(sensors)
