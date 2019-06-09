# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 09:20:24 2018

@author: Mingchen Mao
"""

import pygame as pg
from pygame.sprite import Sprite # for multiple objects


class Ship(Sprite):
    """ Manage the ship object"""
    
    
    def __init__(self, settings, screen):
        super().__init__() # each ship can be operated as a sprite
        
        self.screen = screen # reference for ship to be blit on
        
        # edit the right image for ship
        self.image = pg.image.load('Images\ship.bmp')
        self.image = pg.transform.scale(self.image, settings.ship_size) # pg.transform.scale(Surface, (width, height), DestSurface = None)
    
        # get rect(s) for screen and ship (rect stores position in rectangle format)
        self.screen_rect = screen.get_rect()
        self.rect = self.image.get_rect()
        # ship starts at the center bottom of screen
        self.rect.centerx = self.screen_rect.centerx
        self.rect.bottom = self.screen_rect.bottom
        # convert ship position to calculable format
        self.center = float(self.rect.centerx) 
        self.ship_size = float(settings.ship_size[0]) / 2 # make sure ship does not show half at edges
        
        # get ship dynamics
        self.speed = settings.ship_speed
        
        # movement flag, if key pressed, move
        self.move_right = False
        self.move_left = False
    
    
    def update(self):
        """update ship position based on movement flag"""
        
        # if keydown and ship position within window limit, move
        if self.move_right and self.center < self.screen_rect.right - self.ship_size:
            self.center += self.speed
        if self.move_left and self.center > self.screen_rect.left + self.ship_size:
            self.center -= self.speed
        self.rect.centerx = self.center
       
        
    def blitme(self):
        """blit a ship on the screen.""" 
        
        self.screen.blit(self.image, self.rect) # x.blit(xx, position) = blit xx on x at position
        
    