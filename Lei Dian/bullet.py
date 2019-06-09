# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 10:49:47 2018

@author: Mingchen Mao
"""

import pygame as pg
from pygame.sprite import Sprite # for multiple objects


class Bullet(Sprite):
    """Manage the bullet objects"""
    
    
    def __init__(self, settings, screen, ship):
        super().__init__() # each bullet can be operated as a sprite
        
        self.screen = screen # reference for bullets to be drawn
        self.color = settings.bullet_color
        
        # create a geometry starting at the right position for bullets
        self.rect = pg.Rect(0, 0, settings.bullet_width, settings.bullet_height) # pg.Rect(left, top, width, height)
        self.rect.centerx = ship.rect.centerx
        self.rect.top = ship.rect.top
        
        # convert bullet position to calculable format
        self.yb = float(self.rect.y)
        
        # get bullets dynamics
        self.speed = settings.bullet_speed
        
        
    def update(self):
        """update bullet position"""
        
        self.yb -= self.speed # on top y = 0, bottom y = height
        self.rect.y = self.yb
        
        
    def drawbullet(self):
        """draw a bullet on screen"""
        
        # blit add an image on top of another, draw directly draw on the base image (i.e. screen)
        pg.draw.rect(self.screen, self.color, self.rect) # draw.rect(a,b,c) = draw rectangle c on a
        