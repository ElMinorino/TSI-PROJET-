#!/usr/bin/env python3

from cmath import pi
from enum import auto
from tkinter import Button
import OpenGL.GL as GL
import glfw
from pkg_resources import compatible_platforms
import pyrr
import math
import numpy as np
from cpe3d import Object3D
import random

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
        self.cible = 2
        self.cible_actuelle = None
        self.ListeBriques = self.creer_liste()
        self.debut=True
        self.modif= None

    def run(self):
        # boucle d'affichage
        self.first_update()
        while not glfw.window_should_close(self.window):
            # nettoyage de la fenêtre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)


            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                obj.draw()
            
            if(self.bool):
                self.objs[25].transformation.translation -= \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[25].transformation.rotation_euler), pyrr.Vector3([0, 0, -1]))      
            self.bool=0

            # if sqrt(pow(self.objs[25].transformation.rotation_euler[0] - self.objs[26].transformation.translation[0], 2) + pow(self.objs[25].transformation.rotation_euler[1] - self.objs[26].transformation.translation[1], 2) + pow(self.objs[25].transformation.rotation_euler[2] - self.objs[26].transformation.translation[2], 2)) < self.cible:
            #     self.objs[26].visible = False

            


            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des évènements
            glfw.poll_events()
        
    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action
        
        if key == glfw.KEY_T and action == glfw.PRESS:
            print(self.mouse_x)
        
        if key == glfw.KEY_ENTER and action == glfw.PRESS : 
            self.objs[8+len(self.ListeBriques)].visible = False
            self.objs[9+len(self.ListeBriques)].visible = False
            for i in range(len(self.ListeBriques)-1):
                self.objs[2+i].visible = False 
            self.cible_actuelle = self.objs[1+self.BriqueVisible]
            print(self.ListeBriques)
            
            
        if key == glfw.KEY_S and action == glfw.PRESS:
            self.bool=1
            self.coordXProj = self.mouse_x
            self.coordXCible = pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[0]
            self.coordZCible = pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[2]
            if self.coordXCible <0 and self.coordZCible >-9.5:
                ajout= 0
                self.coordXCible= abs(200*(ajout+np.arcsin(self.coordXCible/12)))   
            elif self.coordZCible <-9.5:
                ajout= np.pi
                self.coordXCible= abs(200*(ajout+np.arcsin(self.coordXCible/12)))
            else:
                ajout = 3*np.pi/2
                self.coordXCible= abs(200*(ajout+np.arccos(self.coordXCible/12)))
            

            print(self.coordXProj,self.coordXCible)  
            if self.coordXProj == self.coordXCible:
                self.cible_actuelle.visible = False
                
    def mouse_button_callback(self,win,button, action,mods):

        if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS :
           

            if self.BriqueVisible == len(self.ListeBriques)-1:
                self.objs[1+self.BriqueVisible].visible = False
                self.BriqueVisible = 0  
            self.objs[1+self.BriqueVisible].visible = False
            self.BriqueVisible +=1
            self.objs[1+self.BriqueVisible].visible = True
            self.cible_actuelle=self.objs[1+self.BriqueVisible]
            print(pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1))
    
    def cursor_position_callback(self, win, xpos, ypos):
        if self.debut:
            self.modif= xpos
            self.debut=False
        xmod=(xpos-self.modif) % 1256
        
        if self.mouse_x != None and self.mouse_y != None:
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += (xmod-self.mouse_x) *0.01/2
            self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] +=  (xmod-self.mouse_x) *0.01/2 
            
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2>=-0.4 and ypos-self.mouse_y <0:
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2
            
            
            
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2<=1 and ypos-self.mouse_y >0:
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2               

       
        self.mouse_x = xmod
        self.mouse_y = ypos
        print(self.mouse_x)
       # print(self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] + (xmod-self.mouse_x) *0.01/2)

   

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
      #    if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
        #     self.cam.transformation.rotation_euler[pyrr.euler.index().roll] -= 0.1
        # if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
        #  AAAAAA   self.objs[0].transformation.translation -= \
        # AAAAAAAAA    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.02]))
        # if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
        #     self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.1
        # if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
        #     self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.1
  

    def first_update(self):
        self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])
    
    def creer_liste(self):
                
        ListeXDec= [i for i in np.arange(-12.1,12,0.1)]

        ListeZDec= [i for i in np.arange(-21.5,2.5,0.1)]

        ListeBriques =[]
        for nbx in ListeXDec:
            for nbz in ListeZDec:
                if abs((nbx)**2+(nbz+9.5)**2-12**2 )<0.05 :
                    nby=random.random()*5+1
                    ListeBriques.append((nbx,nby,nbz))

        
        random.shuffle(ListeBriques)
        return ListeBriques
