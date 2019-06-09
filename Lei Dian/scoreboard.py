# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 22:37:09 2018

@author: Mingchen Mao
"""

from pygame.sprite import Group # store ship left in Group
from ship import Ship


class Scoreboard():
    """Manage the scoreboard object"""
    
    
    def __init__(self, settings, screen, stats):
        self.screen = screen
        self.screen_rect = screen.get_rect()
        self.settings = settings
        self.stats = stats
        
        # create images starting at the right positions for these status
        self.prep_score()
        self.prep_highest_score()
        self.prep_level()
        self.prep_ships()
        self.how_to_play()
        
        
    def prep_score(self):
        """convert the score string to a rendered image and display at top-right corner"""
        
        score_str = int(self.stats.score) # convert to integer
        score_str = 'Score: ' + "{:,}".format(score_str) # 1000 will be displayed as 1,000
        
        # font.render(text, antialias, color, background=None)
        self.image = self.settings.score_font.render(score_str, True, self.settings.score_color, self.settings.color)
        self.image_rect = self.image.get_rect()
        self.image_rect.right = self.screen_rect.right - 20 # 20 gives space for long score
        self.image_rect.top = self.screen_rect.top + 20
       
        
    def prep_highest_score(self):
        """convert the highest score string to a rendered image and display at top center"""
        
        highest_str = int(self.stats.highest)
        highest_str = "{:,}".format(highest_str)
        highest_score_str = 'Highest: ' + highest_str
        
        self.highest_image = self.settings.score_font.render(highest_score_str, True, self.settings.score_color, self.settings.color)
        self.highest_image_rect = self.highest_image.get_rect()
        self.highest_image_rect.centerx = self.screen_rect.centerx
        self.highest_image_rect.top = self.image_rect.top
        
    
    def prep_level(self):
        """convert the level string to a rendered image and display below score"""
        
        level_str = 'Level: ' + str(self.stats.level) # assume no one reaches 1000+ level
        
        self.level_image = self.settings.score_font.render(level_str, True, self.settings.score_color, self.settings.color)
        self.level_image_rect = self.level_image.get_rect()
        self.level_image_rect.right = self.image_rect.right
        self.level_image_rect.top = self.image_rect.top + 20 # 20 gives space between score and level
        
              
    def how_to_play(self):
        """ Display instructions about how to play this game """
        
        
        ins1 = 'Move: arrow keys'
        self.ins1 = self.settings.score_font.render(ins1, True, self.settings.instruction_color, self.settings.color)
        self.ins1_rect = self.ins1.get_rect()
        self.ins1_rect.right = self.level_image_rect.right + 18
        self.ins1_rect.top = self.level_image_rect.top + 40
        ins2 = 'Shoot: w'
        self.ins2 = self.settings.score_font.render(ins2, True, self.settings.instruction_color, self.settings.color)
        self.ins2_rect = self.ins2.get_rect()
        self.ins2_rect.right = self.level_image_rect.right - 48
        self.ins2_rect.top = self.level_image_rect.top + 60
        ins3 = 'Pause: space bar'
        self.ins3 = self.settings.score_font.render(ins3, True, self.settings.instruction_color, self.settings.color)
        self.ins3_rect = self.ins3.get_rect()
        self.ins3_rect.right = self.level_image_rect.right + 18
        self.ins3_rect.top = self.level_image_rect.top + 80
        ins4 = 'Quit: q or click'
        self.ins4 = self.settings.score_font.render(ins4, True, self.settings.instruction_color, self.settings.color)
        self.ins4_rect = self.ins4.get_rect()
        self.ins4_rect.right = self.level_image_rect.right - 5
        self.ins4_rect.top = self.level_image_rect.top + 100
        
        
        
    def prep_ships(self):
        """show how many lives left"""
        
        self.ships = Group()
        
        # display ship left or lives on the top left corner
        for life in range(self.stats.ship_left):
            ship = Ship(self.settings, self.screen) # create an instance for ship left
            ship.rect.x = self.settings.ship_size[0] + 1.5*life * ship.rect.width
            ship.rect.y = self.image_rect.top
            self.ships.add(ship)
            
            
    def blitme(self):
        """blit a scoreboard on the screen""" 
        
        # x.blit(xx, position) = blit xx on x at position
        self.screen.blit(self.image, self.image_rect)
        self.screen.blit(self.highest_image, self.highest_image_rect)
        self.screen.blit(self.level_image, self.level_image_rect)
        self.screen.blit(self.ins1, self.ins1_rect)
        self.screen.blit(self.ins2, self.ins2_rect)
        self.screen.blit(self.ins3, self.ins3_rect)
        self.screen.blit(self.ins4, self.ins4_rect)
        self.ships.draw(self.screen) # pygame.sprite.Group.draw(which surface to draw on)
        
        
    def game_over(self):
        """blit game over on the screen"""
        
        gameover = 'Game Over!'
        
        self.over_image = self.settings.gameover_font.render(gameover, True, self.settings.gameover_color, self.settings.color)
        self.over_image_rect = self.over_image.get_rect()
        self.over_image_rect.centerx = self.screen_rect.centerx
        self.over_image_rect.top = self.highest_image_rect.top + 30
        self.screen.blit(self.over_image, self.over_image_rect)
        