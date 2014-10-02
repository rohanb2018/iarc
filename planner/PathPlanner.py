class PathPlanner(object):

    def __init__(self):
        super(PathPlanner, self).__init__()

    def step(self, world):
    	
        # 3 lists that store quad data, roombas to target and roombas to avoid
        
        quad = []
    	roomba_target = []
    	roomba_avoid = []
    	output = {}
    	length = 200
    	width = 200
    	tick = 1

        # Putting things into different lists
        
    	for key in world:
    		if key == 'quad':
    			quad.append(world[key])
    		else if key == "roomba" and world[key][spike]:
    			roomba_avoid.append(world[key])
    		else:
    			roomba_target.append(world[key])
                
        

    	difference = roomba_target['position'].sub(quad['position'])    # difference between your position and roombas position
    	t = difference.magnitude()/quad['maxvelocity'] 
    	bayesian_t = t * quad['maxvelocity']/roomba_target['velocity'].magnitude()

    	path_o = roomba_target['position'].add(roomba_target['velocity'].scale(bayesian_t / 2))
    	path = roomba_target['position'].add(roomba_target['velocity'].scale(bayesian_t))

    	output['move_%d' % tick] = (path[0], path[1], quad['position'][2]) #(path[0] = x, path[1], quad['position'][2])
    	tick += 1
    	output['descend_%d' % tick] = path # path has z-ccordinate = 0
    	tick += 1

    	if path[0] < length/2: # roomba on left side of grid
    		if roomba_target['velocity'].quadrant() == 2:
    			output['move_%d' % tick] = path.add(roomba_target['velocity'].scale(-1))
    			tick += 1
    			path.add(roomba_target['velocity'].scale(-1))
    			output['ascend_%d' % tick] = (path[0], path[1], 0.7) # ascend to height
    			tick += 1
    			output['descend_%d' % tick] = path.add(roomba_target['velocity'].scale(0.3))
    			tick += 1
    		else if roomba_target['velocity'].quadrant() == 3:
    			output['descend_%d' % (tick - 1)] = path_o
    	else:   # roomba on right side of grid
    		if roomba_target['velocity'].quadrant() == 1:
    			output['descend_%d' % (tick - 1)] = path_o
    		else if roomba_target['velocity'].quadrant() == 3:
    			output['move_%d' % tick] = path.add(roomba_target['velocity'].scale(-1))
    			tick += 1
    			path.add(roomba_target['velocity'].scale(-1))
    			output['ascend_%d' % tick] = (path[0], path[1], 0.7)
    			tick += 1
    			output['descend_%d' % tick] = path.add(roomba_target['velocity'].scale(0.3))
    			tick += 1
        return output





