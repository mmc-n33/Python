# -*- coding: utf-8 -*-
"""
Created on Wed Apr 4 09:40:07 2018

@author: Mingchen Mao
"""

import sys # for exit
import pygame as pg
from bullet import Bullet
from alien import Alien
from time import sleep # for game pause after failure


def check_keydown(event, settings, screen, stats, ship, bullets):
    """ respond to key presses"""
    
    if event.key == pg.K_RIGHT:
        ship.move_right = True # true = move ship
    if event.key == pg.K_LEFT:
        ship.move_left = True 
    if event.key == pg.K_w: # w: shoot
        fire_bullet(settings, screen, stats, ship, bullets)
    if event.key == pg.K_SPACE: # space: pause
        stats.active = False
        stats.paused = True
    if event.key == pg.K_q: # q: exit
        sys.exit()
        
            
def check_keyup(event, ship):
    """ respond to key releases"""
    
    if event.key == pg.K_RIGHT:
        ship.move_right = False # false = do not move ship
    if event.key == pg.K_LEFT:
        ship.move_left = False  
    
        
def check_events(settings, screen, stats, button, score, ship, aliens, bullets):
    """respond to keyboard and mouse inputs"""
    
    for event in pg.event.get(): # get all events in queue
            if event.type == pg.QUIT: # user click close on the window
                sys.exit()
            elif event.type == pg.KEYDOWN: # key was pressed down
                check_keydown(event, settings, screen, stats, ship, bullets)
            elif event.type == pg.KEYUP: # key was released up
                check_keyup(event, ship)
            elif event.type == pg.MOUSEBUTTONDOWN: # mouse was clicked
                mouse_x, mouse_y = pg.mouse.get_pos()
                lets_play(settings, screen, stats, button, score, ship, aliens, bullets, mouse_x, mouse_y)
    
                        
def update_screen(settings, screen, stats, button, score, ship, aliens, bullets):
    """update all kinds of images to screen"""
    
    screen.fill(settings.color) # screen background
       
    ship.blitme() # blit ship
    
    # aliens are sprites, which has a draw command to blit each sprite
    aliens.draw(screen) 

    # bullets are sprites, but are not images, sprite.draw requires image attribute
    for bullet in bullets.sprites():
        bullet.drawbullet() # draw a bullet on screen
    
    score.blitme() # blit scoreboard 
    
    # draw the play button if game just started or lose game
    if not stats.active: # play button only show up when game inactive 
        button.drawbutton() # draw a button on screen
        score.game_over()
        pg.mouse.set_visible(True) # mouse is visible so that user can click
        
    # tells Pygame to make the most recently drawn screen visible
    pg.display.flip()
    
    # check if any alien hits your ship (you lose)
    if pg.sprite.spritecollideany(ship, aliens): # check if ship collides with any alien
        after_lost(settings, screen, stats, score, ship, aliens, bullets)
    # check if any alien reaches the screen bottom (you lose)
    screen_rect = screen.get_rect()
    for alien in aliens.sprites():
        if alien.rect.bottom >= screen_rect.bottom:
            after_lost(settings, screen, stats, score, ship, aliens, bullets)
            break


def update_aliens(settings, screen, stats, score, aliens, bullets):
    """update position of aliens based on their positions and numbers"""
    
    # everytime an alien hits any edge, all aliens get dropped down and move towards an opposite direction
    for alien in aliens.sprites():
        if alien.hit_edges():
            for alien2 in aliens.sprites():
                alien2.rect.y += settings.alien_drop_speed # drop aliens 1 level down
            settings.alien_direction *= -1 # change aliens direction
            break
    
    # update aliens positions
    aliens.update()
    
    # if aliens are all eliminated, increase speed/level and repopulate them
    if len(aliens) == 0:
        settings.speed_up() # increase the overall game speed (every speed)
        stats.level += 1 # reach a higher level
        score.prep_level() # display this new level
        
        bullets.empty() # clear bullets left on screen
        create_aliens(settings, screen, aliens) # repopulate a full fleet of aliens

    
def update_bullets(settings, stats, score, aliens, bullets):
    """update position of bullets and get rid of old/collided bullets"""
    
    # update bullets positions
    bullets.update() 
    
    # if bullets get outside the screen, delete them
    for bullet in bullets.copy(): # sprite.group.copy() creates a copy of the original group, so that get to check all bullets
        if bullet.rect.bottom <= 0:
            bullets.remove(bullet) # remove from the original group
            
    # if a bullet hits aliens, get corresponding score
    collisions = pg.sprite.groupcollide(bullets, aliens, True, True) # groupcollide(group1, group2, dokill1, dokill2)
    if collisions:
        for aliens in collisions.values(): # values are the collided group2 
            stats.score += settings.alien_points * len(aliens) # if 1 bullet hits multiple aliens, get multiple points
            score.prep_score() # display new score
        update_highest_score(stats, score) # display highest score if exceeds the old one


def lets_play(settings, screen, stats, button, score, ship, aliens, bullets, mouse_x, mouse_y):
    """start a new game when user clickes the play button"""
    
    # if button clicked, play, stats being set active, so click again this area (although button not displayed), no effect
    if button.rect.collidepoint(mouse_x, mouse_y) and not stats.active:
        # if paused, just resume game
        pg.mouse.set_visible(False) # hide cursor
        stats.active = True # begin game
        
        # if not paused, reset
        if not stats.paused:
            settings.initialize_speed() # initialize game speed
            stats.reset_stats() # initialize score, level, life
        
            # display these things
            score.prep_score()
            score.prep_highest_score()
            score.prep_level()
            score.prep_ships()
            score.how_to_play()
        
            # empty existing aliens and bullets, create a new full fleet of aliens
            aliens.empty()
            bullets.empty()
            create_aliens(settings, screen, aliens)
        
            # reset position of ship
            screen_rect = screen.get_rect()
            ship.rect.centerx = screen_rect.centerx
            
        stats.paused = False 

def update_highest_score(stats, score):
    """check if current score higher than the highest"""

    if stats.score > stats.highest:
        stats.highest = stats.score
        score.prep_highest_score()
        with open('highest score.txt', 'w') as high: # 'w': write mode
            stats.highest = high.write(str(stats.highest)) # can only write string in
        
        
def create_aliens(settings, screen, aliens):
    """create a full fleet of aliens at game start or restart"""

    # assume the space between aliens = alien width
    alien_edge = (settings.width - (2*settings.alien_row - 1) * settings.alien_size[0]) / 2 
    
    for j in range(1, settings.alien_column + 1): # column
        for i in range(1, settings.alien_row + 1): # row
            alien = Alien(settings, screen) # create an instance from Alien
            alien.xa = alien_edge + 2*(i - 1)*settings.alien_size[0]
            alien.ya += alien.ya + 2*(j - 1)*settings.alien_size[1]
            alien.rect.x = alien.xa
            alien.rect.y = alien.ya
            aliens.add(alien)
    
    
def fire_bullet(settings, screen, stats, ship, bullets):
    """create a bullet if # of bullets < allowed"""
    
    if len(bullets) < settings.bullet_allowed:
            new_bullet = Bullet(settings, screen, ship) # create an instance from Bullet
            bullets.add(new_bullet) # add this new bullet to the bullets group
            

def after_lost(settings, screen, stats, score, ship, aliens, bullets):
    """includes what to do if any alien hits ship or the screen bottom"""
    
    
    if stats.ship_left > 0:  # game not over
        # decrement life by 1
        stats.ship_left -= 1
        
        # update life display
        score.prep_ships()
        
        # empty existing aliens and bullets, create a new full fleet of aliens
        aliens.empty()
        bullets.empty()
        create_aliens(settings, screen, aliens)
        
        # reset position of ship
        screen_rect = screen.get_rect()
        ship.rect.centerx = screen_rect.centerx
        
        # pause for 1 second
        sleep(1)
        
    else:  # game over
        stats.active = False # game becomes inactive, play button will show up
    
    

        
    
        