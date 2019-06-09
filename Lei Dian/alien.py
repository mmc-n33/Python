# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 12:50:12 2018

@author: Mingchen Mao
"""

import pygame as pg
from pygame.sprite import Sprite # for multiple objects


class Alien(Sprite):
    """Manage the alien objects"""
    
    
    def __init__(self, settings, screen):
        super().__init__() # each alien can be operated as a sprite
        
        self.screen = screen # reference for aliens to be blit on
        
        # edit the right image for aliens
        self.image = pg.image.load('Images\guai.bmp') # \a does not work?, so did not use \alien.bmp
        self.image = pg.transform.scale(self.image, settings.alien_size) # pg.transform.scale(Surface, (width, height), DestSurface = None)
        
        # get rect(s) for screen and aliens (rect stores position in rectangle format)
        self.screen_rect = self.screen.get_rect()
        self.rect = self.image.get_rect()
        # the initial position of an alien is at (width, height)
        self.rect.x = self.rect.width
        self.rect.y = self.rect.height / 2
        # convert alien position to calculable format
        self.xa = float(self.rect.x)
        self.ya = float(self.rect.y)
        
        # get aliens dynamics
        # speed does not change but direction changes, if define a variable to store direction info, it will 
        # remain this value in below methods, since we will change settings.direction, not alien.this variable
        self.speed = settings.alien_speed
        self.settings = settings
    
    
    def hit_edges(self):
        """check if alien fleets hit edges"""
        
        if self.rect.right >= self.screen_rect.right:
            return True
        elif self.rect.left <= self.screen_rect.left:
            return True
        
        
    def update(self):
        """update aliens position"""
        
        self.xa += self.speed * self.settings.alien_direction
        self.rect.x = self.xa
     
        
    def blitme(self):
        """blit a alien on the screen.""" 
        
        self.screen.blit(self.image, self.rect) # x.blit(xx, position) = blit xx on x at position
        