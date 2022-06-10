#!/usr/bin/env python3

from enum import auto
from tkinter import Button
import OpenGL.GL as GL
import glfw
import pyrr
import math
import numpy as np
from cpe3d import Object3D

class ViewerGL:
    BriqueVisible=0
    def __init__(self):
        # initialisation de la librairie GLFW
        glfw.init()
        # paramétrage du context OpenGL
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL.GL_TRUE)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        # création et paramétrage de la fenêtre
        glfw.window_hint(glfw.RESIZABLE, False)
        self.window = glfw.create_window(1200, 1200, 'OpenGL', None, None)

        glfw.set_input_mode(self.window,glfw.CURSOR,glfw.CURSOR_DISABLED)
        # paramétrage de la fonction de gestion des évènements
        glfw.set_key_callback(self.window, self.key_callback)
        glfw.set_mouse_button_callback(self.window, self.mouse_button_callback)
        glfw.set_cursor_pos_callback(self.window,self.cursor_position_callback)
        # activation du context OpenGL pour la fenêtre
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        # activation de la gestion de la profondeur
        GL.glEnable(GL.GL_DEPTH_TEST)
        # choix de la couleur de fond
        GL.glClearColor(0.5, 0.6, 0.9, 1.0)
        print(f"OpenGL: {GL.glGetString(GL.GL_VERSION).decode('ascii')}")

        self.objs = []
        self.touch = {}
        self.bool = 0
        self.mouse_x= None
        self.mouse_y = None

    # def new_target(self):
    #     self.objs[5+self.i]

    def run(self):
        # boucle d'affichage
        self.first_update()
        while not glfw.window_should_close(self.window):
            # nettoyage de la fenêtre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            
         #  self.update_key()

            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                obj.draw()

            if(self.bool):
                for i in range(18):
                    self.objs[24].transformation.translation.z +=0.5
                    
                    
                self.bool=0

            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des évènements
            glfw.poll_events()
        
    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action
        
        if key == glfw.KEY_ENTER and action == glfw.PRESS : 
            self.objs[1].visible = False
            self.objs[2].visible = False
            for i in range(18):
                self.objs[4+i].visible = False 
                

    
        if key == glfw.KEY_S and action == glfw.PRESS:
            self.bool=1

    def mouse_button_callback(self,win,button, action,mods):          
        if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS :

            if self.BriqueVisible == 18:
                self.objs[3+self.BriqueVisible].visible = False
                self.BriqueVisible = 0  
            self.objs[3+self.BriqueVisible].visible = False
            self.BriqueVisible +=1
            self.objs[3+self.BriqueVisible].visible = True
    
    
    def cursor_position_callback(self, win, xpos, ypos):

        if self.mouse_x != None and self.mouse_y != None:

            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += (xpos-self.mouse_x) *0.01/2
            
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2>=-0.4 and ypos-self.mouse_y <0:
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2
                
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2<=1 and ypos-self.mouse_y >0:
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2               
        
        self.mouse_x = xpos
        self.mouse_y = ypos

    def add_object(self, obj):
        self.objs.append(obj)

    def set_camera(self, cam):
        self.cam = cam

    def update_camera(self, prog):
        GL.glUseProgram(prog)
        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "translation_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : translation_view")
        # Modifie la variable pour le programme courant
        translation = -self.cam.transformation.translation
        GL.glUniform4f(loc, translation.x, translation.y, translation.z, 0)

        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "rotation_center_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_center_view")
        # Modifie la variable pour le programme courant
        rotation_center = self.cam.transformation.rotation_center
        GL.glUniform4f(loc, rotation_center.x, rotation_center.y, rotation_center.z, 0)

        rot = pyrr.matrix44.create_from_eulers(-self.cam.transformation.rotation_euler)
        loc = GL.glGetUniformLocation(prog, "rotation_view")
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_view")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, rot)
    
        loc = GL.glGetUniformLocation(prog, "projection")
        if (loc == -1) :
            print("Pas de variable uniforme : projection")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, self.cam.projection)

   # def update_key(self):
        # if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
        #     self.cam.transformation.rotation_euler[pyrr.euler.index().roll] -= 0.1
        # if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
        #     self.objs[0].transformation.translation -= \
        #     pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.02]))
        # if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
        #     self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.1
        # if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
        #     self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.1



        
                

    def first_update(self):
        self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])
