# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 08:53:50 2018

@author: Mingchen Mao
"""

# import pygame modules
import pygame as pg
from pygame.sprite import Group

# import my modules
import gamefunctions as gf
from settings import Settings
from gamestats import GameStats
from button import Button
from scoreboard import Scoreboard
from ship import Ship

"""game over should not show during pause ----- things can be modified"""

def gamerun():
    """call this function to run the game"""
    
    pg.init() # initialize pygame
    
    # "the static part" of this game
    settings = Settings()
    # pygame.display.set_mode(resolution) = initialize a window or screen for display
    screen = pg.display.set_mode((settings.width, settings.height))
    pg.display.set_caption(settings.title) 
    

    stats = GameStats(settings)
    button = Button(settings, screen, 'Play')
    score = Scoreboard(settings, screen, stats)
    ship = Ship(settings, screen)
    bullets = Group()
    aliens = Group()
    
    
    # "the dynamic part" of this game
    while True: 
        # check user input and respond correspondingly
        gf.check_events(settings, screen, stats, button, score, ship, aliens, bullets)
        
        # update ship, aliens and bullets status
        if stats.active == True:
            ship.update()
            gf.update_aliens(settings, screen, stats, score, aliens, bullets)
            gf.update_bullets(settings, stats, score, aliens, bullets)
        
        # display updates
        gf.update_screen(settings, screen, stats, button, score, ship, aliens, bullets)       

        
gamerun()