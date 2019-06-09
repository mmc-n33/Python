# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 20:15:20 2018

@author: Mingchen Mao
"""
 
class GameStats():
    """Store and respond to game statistics"""
    
    
    def __init__(self, settings):
        self.settings = settings
        self.active = False # only effect the first time game starts
        self.paused = False # only effect the first time game starts
        
        # open the txt file which stores the highest score
        with open('highest score.txt', 'r') as high: # 'r': read mode
            self.highest = int(high.read()) # stored as a string, convert to integer
        
        # clear previous score and level, renew lives
        self.reset_stats()
        
        
    def reset_stats(self):
        """initialize stats (life, score, level)"""
        
        self.ship_left = self.settings.ship_life
        self.score = 0
        self.level = 1
        
            