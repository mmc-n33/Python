# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 09:03:36 2018

@author: Mingchen Mao
"""
import pygame.font


class Settings():
    """Store all settings for "Lei Dian"."""
    
    
    def __init__(self):
        
        # screen settings
        self.width = 600
        self.height = 400
        self.color = (200, 200, 0)
        self.title = 'Lei Dian'
        
        # button settings
        self.button_width = 100
        self.button_height = 50
        self.button_color = (200, 200, 200)
        self.text_color = (0, 0, 0)
        self.text_font = pygame.font.SysFont('arial', 24)
        
        # scoreboard settings
        self.score_color = (255, 255, 255)
        self.score_font = pygame.font.SysFont(None, 24)
        self.instruction_color = (0, 0, 0)
        self.gameover_color = (255, 0, 0)
        self.gameover_font = pygame.font.SysFont(None, 72)
        
        # ship settings 
        self.ship_size = (30, 30)
        self.ship_life = 3 # how many ships can be hit
        
        # bullet settings
        self.bullet_width = 3
        self.bullet_height = 8
        self.bullet_color = (255, 1, 1)
        self.bullet_allowed = 3
        
        # alien settings
        self.alien_size = (30, 30)
        self.alien_row = 7 # # of aliens in a row
        self.alien_column = 4
        
        # game speed up settings
        self.speedup = 1.2
        
        # initialize settings
        self.initialize_speed()
        
        
    def initialize_speed(self):
        """initialize speed settings"""
        
        self.ship_speed = 0.15
        self.bullet_speed = 0.3
        self.alien_speed = 0.15
        self.alien_drop_speed = 10
        self.alien_direction = 1 # 1 means right, -1 means left
        self.alien_points = 50 # how many points 1 alien worth
        

    def speed_up(self):
        """increase overall game speed if kill all aliens"""
        
        self.ship_speed *= self.speedup
        self.bullet_speed *= self.speedup
        self.alien_speed *= self.speedup
        self.alien_drop_speed *= self.speedup
        self.alien_points = int(self.alien_points * self.speedup)