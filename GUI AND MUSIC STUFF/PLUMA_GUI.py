#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  2 22:14:37 2022

@author: pedrotokushiro
"""

import pygame

pygame.font.init()



'''
We will have to initialize certain parameters in the GUI, such as the screen size that we want, and load certain 
pictures that we will use as the front end buttons the user presses
'''
# These are RGB colors that pygame uses, and we can name a couple of them for ease of use
GREY = (70,70,70)
WHITE = (255,255,255)

# This is the value at which we will have the GUI update 
FPS = 60
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 700

# Initialize the window
WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Clotune GUI')

# The size od the images that we will use shortly 
BUTTON_WIDTH = 70
BUTTON_HEIGHT = 70
FLEX_WIDTH = 50
FLEX_HEIGHT = 50

#Load in the images that we will use as the buttons and sliders
BACKGROUND_IMAGE = pygame.image.load('BACKGROUND.jpeg')
BUTTON1_ON_IMAGE = pygame.image.load('BUTTON_ON.png')
BUTTON1_OFF_IMAGE = pygame.image.load('BUTTON_OFF.png')
FLEX_IMAGE = pygame.image.load('FLEX.jpeg').convert_alpha()
SLIDER_IMAGE = pygame.image.load('SLIDER.jpeg').convert_alpha()

# Transform every lodaed image to the size that we want
SLIDER = pygame.transform.scale(SLIDER_IMAGE, (50, 200))
BACKGROUND = pygame.transform.scale(BACKGROUND_IMAGE, (SCREEN_WIDTH, SCREEN_HEIGHT))
BUTTON1_ON = pygame.transform.scale(BUTTON1_ON_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
BUTTON1_OFF = pygame.transform.scale(BUTTON1_OFF_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
FLEX = pygame.transform.scale(FLEX_IMAGE, (FLEX_WIDTH, FLEX_HEIGHT))

# Border for the slider
BORDER = pygame.Rect(0, SCREEN_HEIGHT/2.5, 600, 700)
slider1_border = pygame.Rect(700, 350, 50, 200) # flex sensor 1 
slider2_border = pygame.Rect(800, 350, 50, 200) # flex sensor 2
slider3_border = pygame.Rect(100, 350, 200, 50) # accel sensor 1
slider4_border = pygame.Rect(100, 450, 200, 50) # accel sensor 2
slider5_border = pygame.Rect(100, 550, 200, 50) # who knows at this point

#Get the font that we want for the Captions
title_font = pygame.font.SysFont('americantypewriter', 150)
button_font = pygame.font.SysFont('americantypewriter', 20)

current_button1 = BUTTON1_ON
current_button2 = BUTTON1_OFF
current_button3 = BUTTON1_OFF



'''We will have a draw window function , in which it will update the screen and every values needed'''
def draw_window(flex1, flex2,accel1, accel2, accel3, current_button1, current_button2, current_button3):
    #WIN.blit(BACKGROUND, (0,0)) # This fills the screen with white as the numbers indicate RGB, with a range of 0-255
    clotune_text = title_font.render('CLOTUNE', 300, GREY)
    button1_text = button_font.render('Track 1', 500, WHITE)
    button2_text = button_font.render('Track 2', 500, WHITE)
    button3_text = button_font.render('Track 3', 500, WHITE)
    flex1_text = button_font.render('Flex 1', 500, WHITE)
    flex2_text = button_font.render('Flex 2', 500, WHITE)
    WIN.blit(clotune_text, (150, 50))
    WIN.blit(button1_text, (900, 375))
    WIN.blit(button2_text, (900, 475))
    WIN.blit(button3_text, (900, 575))
    WIN.blit(flex1_text, (700, 325))
    WIN.blit(flex2_text, (800, 325))
    pygame.draw.rect(WIN,WHITE, slider1_border)
    pygame.draw.rect(WIN,WHITE, slider2_border)
    pygame.draw.rect(WIN,GREY, slider3_border)
    pygame.draw.rect(WIN,GREY, slider4_border)
    pygame.draw.rect(WIN,GREY, slider5_border)
    WIN.blit(current_button1, (900,400))
    WIN.blit(current_button2, (900,500))
    WIN.blit(current_button3, (900,600))
    WIN.blit(FLEX, (700, flex1.y))
    WIN.blit(FLEX, (800, flex2.y))
    WIN.blit(FLEX, (accel1.x, 350))
    WIN.blit(FLEX, (accel2.x, 450))
    WIN.blit(FLEX, (accel3.x, 550))
    pygame.display.update()


# class Button():
#     def __init__(self,x,y,image):
#         self.image = image
#         self.rect = self.image.get_rect()
#         self/rect.topleft = (x,y)
#         self.clicked = False
        
#     def draw(self):
        
#         # get mouse position
#         pos = pygame.mouse.get_pos()
        
        
#         # check mouseover and clicked conditions
#         if pygame.mouse.get_pressed()[0] == 1 and self.clicked == False:
#             self.clicked = True
#         if pygame.mouse.get_pressed()[0] == 
        
#         WIN.blit(self.image,(self.rect.x, self.rect.y))

# def button_press(keys_pressed, button1):
#     clicked = False
#     pos = pygame.mouse.get_pos()
#     # check mouseover and clicked conditions
#     if pygame.mouse.get_pressed()[0] == 1 and clicked == False:
#         clicked = True
#         WIN.blit(BUTTON1_ON, (900,400))
#         print('yo mama')
#     # if pygame.mouse.get_pressed()[0] == 1 and clicked == True:
#     #     clicked = False
#     #     WIN.blit(BUTTON1_OFF, (900,400))
#     #     print('is a hoe')

        
 
        
        
def flex1_handle_movement(keys_pressed, flex1):
    if keys_pressed[pygame.K_w] and flex1.y > 350 : # UP
        flex1.y -= 5
    if keys_pressed[pygame.K_s] and flex1.y + FLEX_HEIGHT < 550:
        flex1.y += 5

def flex2_handle_movement(keys_pressed, flex2):
    if keys_pressed[pygame.K_UP] and flex2.y > 350: # LEFT
        flex2.y -= 5
    if keys_pressed[pygame.K_DOWN] and flex2.y + FLEX_HEIGHT < 550:
        flex2.y += 5


def accel1_handle_movement(keys_pressed, accel1):
    if keys_pressed[pygame.K_LEFT] and accel1.x > 100: # LEFT
        accel1.x -= 5
    if keys_pressed[pygame.K_RIGHT] and accel1.x + FLEX_WIDTH < 300:
        accel1.x += 5

def accel2_handle_movement(keys_pressed, accel2):
    if keys_pressed[pygame.K_a] and accel2.x > 100: # LEFT
        accel2.x -= 5
    if keys_pressed[pygame.K_d] and accel2.x + FLEX_WIDTH < 300:
        accel2.x += 5

def accel3_handle_movement(keys_pressed, accel3):
    if keys_pressed[pygame.K_COMMA] and accel3.x > 100: # LEFT
        accel3.x -= 5
    if keys_pressed[pygame.K_PERIOD] and accel3.x + FLEX_WIDTH < 300:
        accel3.x += 5


def main():
    flex1 = pygame.Rect(700, 450, FLEX_WIDTH, FLEX_HEIGHT)
    flex2 = pygame.Rect(800, 450,FLEX_WIDTH, FLEX_HEIGHT ) 
    accel1 = pygame.Rect(150, 350,FLEX_WIDTH, FLEX_HEIGHT ) 
    accel2 = pygame.Rect(150, 350,FLEX_WIDTH, FLEX_HEIGHT )
    accel3 = pygame.Rect(150, 350,FLEX_WIDTH, FLEX_HEIGHT )
    clock = pygame.time.Clock()
    i = 0
    global current_button1, current_button2, current_button3
    run = True
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx,my = pygame.mouse.get_pos()
                if my > 400 and my < 470 and mx > 900 and mx < 970:
                    if current_button1 == BUTTON1_ON:
                        current_button1 = BUTTON1_OFF
                    else:
                        current_button1 = BUTTON1_ON
                if my > 500 and my < 570 and mx > 900 and mx < 970:
                    if current_button2 == BUTTON1_OFF:
                        current_button2 = BUTTON1_ON
                    else:
                        current_button2 = BUTTON1_OFF       
                if my > 600 and my < 670 and mx > 900 and mx < 970:
                    if current_button3 == BUTTON1_OFF:
                        current_button3 = BUTTON1_ON
                    else:
                        current_button3 = BUTTON1_OFF                    
                
        if pygame.key.get_pressed()[pygame.K_ESCAPE]:
            run = False
        
        keys_pressed = pygame.key.get_pressed()
        flex1_handle_movement(keys_pressed, flex1)
        flex2_handle_movement(keys_pressed, flex2)
        accel1_handle_movement(keys_pressed, accel1)
        accel2_handle_movement(keys_pressed, accel2)
        accel3_handle_movement(keys_pressed, accel3)
        # button_press(keys_pressed, button1)
        WIN.blit(BACKGROUND, (i, 0))
        WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
        if i == -SCREEN_WIDTH:
            WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
            i = 0
        i -= 1        
        draw_window(flex1, flex2, accel1, accel2, accel3, current_button1, current_button2,current_button3)
        



if __name__ == "__main__":
    main()

#     main()