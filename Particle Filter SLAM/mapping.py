# -*- coding: utf-8 -*-
"""
Created on Tue Feb 19 15:23:56 2019

@author: Mingchen Mao
"""

import numpy as np

from parameters import Parameters
para = Parameters()



class Mapping():
    """ Mapping class, so that the log-odds map can be treated as an object """
    
    def __init__(self, para):
                       
        self.cell_num = para.cell_num
        self.cell_size = para.cell_size
        self.trust_1 = para.trust_1 
        self.trust_0 = para.trust_0
        self.log_odds = np.log(0.5) * np.ones((self.cell_num, self.cell_num)) # unknown cell has equal chance to be anything
        


    def Bresenham2D(self, start, end):
        """ Bresenham's Line Algorithm
        Produces a list of tuples from start and end
     
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        
        http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
        """
        
        
        # Setup initial conditions
        x1, y1 = int(round(start[0])), int(round(start[1])) # convert coordinates to integers
        x2, y2 = int(round(end[0])), int(round(end[1]))
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
         
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
         
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
         
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
         
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
         
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
         
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        
        return points


    
    def update__cells(self, points):
        """ update log-odd of each cell based on if there are points passing by
        
        Inputs:
            points = output from Bresenham algorithm, list of tuples with (x, y) coordinates 
        Outputs:
            self.log_odds = log-odds map object """
    

        # Update free cells
        self.log_odds[[int(-py[1] + (self.cell_num - 1) / 2) for py in points[0 : -1]], 
                  [int(px[0] + (self.cell_num - 1) / 2) for px in points[0 : -1]]] += np.log(self.trust_0)

        # Update occupied cells
        self.log_odds[int((-points[-1][1] + (self.cell_num - 1) / 2)),  
                      int((points[-1][0] + (self.cell_num - 1) / 2))] += np.log(self.trust_1) 
        
        return self.log_odds
    
