#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  2 17:07:10 2022

@author: pedrotokushiro
"""

import pygame

pygame.font.init()

# print(pygame.font.get_fonts())

GREY = (70,70,70)
WHITE = (255,255,255)
FPS = 60
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 700
WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Clotune GUI')

BUTTON_WIDTH = 70
BUTTON_HEIGHT = 70
FLEX_WIDTH = 50
FLEX_HEIGHT = 50

BACKGROUND_IMAGE = pygame.image.load('BACKGROUND.jpeg')
BUTTON1_ON_IMAGE = pygame.image.load('BUTTON_ON.png')
BUTTON1_OFF_IMAGE = pygame.image.load('BUTTON_OFF.png')
FLEX_IMAGE = pygame.image.load('FLEX.jpeg').convert_alpha()
SLIDER_IMAGE = pygame.image.load('SLIDER.jpeg').convert_alpha()

SLIDER = pygame.transform.scale(SLIDER_IMAGE, (50, 200))
BACKGROUND = pygame.transform.scale(BACKGROUND_IMAGE, (SCREEN_WIDTH, SCREEN_HEIGHT))
BUTTON1_ON = pygame.transform.scale(BUTTON1_ON_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
BUTTON1_OFF = pygame.transform.scale(BUTTON1_OFF_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
FLEX = pygame.transform.scale(FLEX_IMAGE, (FLEX_WIDTH, FLEX_HEIGHT))

BORDER = pygame.Rect(0, SCREEN_HEIGHT/2.5, 600, 700)
slider1_border = pygame.Rect(700, 350, 50, 200)
slider2_border = pygame.Rect(800, 350, 50, 200)

title_font = pygame.font.SysFont('americantypewriter', 150)
button_font = pygame.font.SysFont('americantypewriter', 20)


# pygame.Rect(left, top, width, height)
def draw_window(flex1, flex2, current_button1, current_button2, current_button3):
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
    WIN.blit(current_button1, (900,400))
    WIN.blit(current_button2, (900,500))
    WIN.blit(current_button3, (900,600))
    WIN.blit(FLEX, (700, flex1.y))
    WIN.blit(FLEX, (800, flex2.y))
    


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

def mouse_click(mx,my,current_button1,current_button2,current_button3):
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

width = 1024


def main():
    flex1 = pygame.Rect(700, 450, FLEX_WIDTH, FLEX_HEIGHT)
    flex2 = pygame.Rect(800, 450,FLEX_WIDTH, FLEX_HEIGHT ) 
    clock = pygame.time.Clock()
    current_button1 = BUTTON1_ON
    current_button2 = BUTTON1_OFF
    current_button3 = BUTTON1_OFF
    i = 0
    run = True
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx,my = pygame.mouse.get_pos()
                mouse_click(mx, my, current_button1, current_button2, current_button3)
        
        if pygame.key.get_pressed()[pygame.K_ESCAPE]:
            run = False
        
        keys_pressed = pygame.key.get_pressed()
        flex1_handle_movement(keys_pressed, flex1)
        flex2_handle_movement(keys_pressed, flex2)
        # button_press(keys_pressed, button1)
        WIN.blit(BACKGROUND, (i, 0))
        WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
        if i == -SCREEN_WIDTH:
            WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
            i = 0
        i -= 1

        draw_window(flex1, flex2, current_button1, current_button2,current_button3)
        

     
        pygame.display.update()


if __name__ == "__main__":
    main()


# '''
# Let's Create the 

# '''
# class ApplicationGlobal:
#     def __init__(self):
#         self.screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
#         pygame.display.set_caption("Clotune GUI")
#         self.status = True
#         self.App = None
#         self.flag = False
#         self.lastpresssed = False
#         self.fullscreen = 0
#     def SetNewApp(self , app):
#         self.App = app
#         self.App.headGlobal = self
#         self.App.screen = self.screen
#     def Refresh(self):
#         self.screen.fill((255,255,255))
#         self.App.Refresh()
#         pygame.display.update()
#         if not pygame.key.get_pressed()[pygame.K_LCTRL]:
#             self.flag = False
#         if not pygame.key.get_pressed()[pygame.K_LCTRL]:
#             self.flag = False
#         if pygame.key.get_pressed()[pygame.K_LCTRL] and pygame.key.get_pressed()[pygame.K_s] and not self.flag:
#             self.flag = True
#             name =  str( time.time() )+ ".png"
#             pygame.image.save(self.screen , str( time.time() )+ ".png")
#             print("screenshot ",name," saved")

#         if pygame.key.get_pressed()[ pygame.K_F11 ] and not self.lastpresssed:
#             self.lastpresssed = True
#             if self.fullscreen == 0:
#                 self.screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT),pygame.FULLSCREEN)
#                 self.fullscreen = 1
#             else:
#                 self.fullscreen = 0
#                 self.screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))

#         if not pygame.key.get_pressed()[pygame.K_F11]:
#             self.lastpresssed  = False
#     def Quit(self):
#         self.App.Kill()
#         self.status = False
        


# class BasicApp:
#     def __init__(self):
#         self.headGlobal = None
#         self.screen = None

#     def Refresh(self):
#         pass
#     def QuitFor(self, app):
#         if app == None:
#             self.headGlobal.Quit()
#         else:
#             self.headGlobal.SetNewApp( app )
#     def Kill(self):
#         pass


# class TestApp(BasicApp):
#     def __init__(self):
#         BasicApp.__init__(self)
#         self.status = 0
#         self.timer = Timer()
#         self.timer.Play()

#         self.showText = []

#         self.show1 = False 
#         self.show2 = False
#         self.show3 = False 
#         self.show3pos = 0
#         self.whiteCycle = False

#     def Refresh(self):
#         if not self.whiteCycle:
#             self.screen.fill((20,20,20))
#         else:
#             grey = self.timer.MS() *200 / 1000
#             if grey > 200:
#                 grey = 200
#                 self.QuitFor(MainWindow())
#             self.screen.fill((grey,grey,grey))
#         if self.status == 0:
#             if self.timer.MS() >= 200:
#                 self.status = 1
#         elif self.status == 1:
#             self.show1 = True
#             if self.timer.MS() >= 400:
#                 self.status = 2
#                 self.timer.Reset()
#                 self.timer.Play()
#         elif self.status == 2:
#             self.show2 = True
            
#             if self.timer.MS() >= 1000:
#                 self.status = 3
#                 self.timer.Reset()
#                 self.timer.Play()
#                 self.show3 = 1
#                 self.show3pos = -self.timer.MS()+SCREEN_WIDTH
#         elif self.status == 3:
#             self.show3pos = -self.timer.MS()+SCREEN_WIDTH
#             if self.show3pos < 30:
#                 self.status = 4
#                 self.timer.Reset()
#                 self.timer.Play()
#         elif self.status == 4:
#             self.whiteCycle = True
#         if self.show1:
#             text = fonts.adamCG[str(110)].render("Clo",1,(200,200,200))
#             BlitInFirstQuarter(self.screen,text)
#         if self.show2:
#             text2 = fonts.adamCG[str(110)].render("Tune",1,(200,200,200))
#             BlitInThirdQuarter(self.screen,text2)
#         if self.show3:
#             text3 = fonts.adamCG[str(40)].render("Pedro - Radu - Cornelius ",1,(200,200,200))
#             BlitInCenterY(self.screen,text3,self.show3pos)


# # class MainWindow(BasicApp):
# #     def __init__(self):
# #         BasicApp.__init__(self)
# #         self.status = 0
# #         self.timer = Timer()
# #         self.timer.Play()

# #         self.showText = []

# #         self.show1 = False 
# #         self.show2 = False
# #         self.show3 = False 
# #         self.show3pos = 0
# #         self.whiteCycle = False
# #         self.screen.fill((200,200,200))
# #         text = fonts.adamCG[str(40)].render("Welcome to Clotune",1,(0,0,0))
# #         BlitInFirstQuarterY(self.screen,text)
    

# FPS = 60

# intialApp = TestApp()
# # window = MainWindow()

# def main():
#     app = ApplicationGlobal()
#     app.SetNewApp ( intialApp )
#     clock = pygame.time.Clock()
#     while app.status:
#         clock.tick(FPS)
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 app.Quit()
#         if pygame.key.get_pressed()[pygame.K_ESCAPE]:
#             app.Quit()

#         app.Refresh()


# if __name__ == "__main__":
#     main()