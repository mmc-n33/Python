# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 20:44:47 2018

@author: Mingchen Mao
"""

import pygame as pg


class Button():
    """Manage a button object"""
    
    
    def __init__(self, settings, screen, msg):
        self.screen = screen # reference for button to be drawn and text to be blit
        self.settings = settings

        # create a geometry starting at the right position for button
        self.rect = pg.Rect(0, 0, settings.button_width, settings.button_height) # pg.Rect(left, top, width, height)
        self.screen_rect = screen.get_rect()
        self.rect.center = self.screen_rect.center
        
        # create an image starting at the right position for text
        self.prep_msg(msg)
        
    
    def prep_msg(self, msg):
        """convert the message string to a rendered image and display at center"""
        
        # font.render(text, antialias, color, background=None)
        self.image = self.settings.text_font.render(msg, True, self.settings.text_color, self.settings.button_color)
        self.image_rect = self.image.get_rect()
        self.image_rect.center = self.rect.center
        
        
    def drawbutton(self):
        """draw a button and blit text on screen"""
        
        pg.draw.rect(self.screen, self.settings.button_color, self.rect) # draw.rect(a,b,c) = draw rectangle c on a
        self.screen.blit(self.image, self.image_rect) # x.blit(xx, position) = blit xx on x at position
        
        