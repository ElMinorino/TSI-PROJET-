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
import time

class ViewerGL:
    
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
        self.window = glfw.create_window(1600, 1200, 'OpenGL', None, None) 

        glfw.set_input_mode(self.window,glfw.CURSOR,glfw.CURSOR_DISABLED) # Pour ne pas avoir de curseur
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
        self.textes=[]
        self.decors=[]
        self.touch = {}
        self.bool = 0
        self.mouse_x= 0
        self.mouse_y = 0
        self.cible_actuelle = None
        self.ListeBriques = self.creer_liste()
        self.debut=True
        self.modif= None
        self.points=0
        self.tirs= 0
        self.debut_partie=0
        self.y_traite=0
        self.valx=0
        self.mode = None
        self.demarrage= False 
        self.BriqueVisible= 0 
        
    
    def run(self):
        # boucle d'affichage
        self.first_update()

        while not glfw.window_should_close(self.window):
            # nettoyage de la fenêtre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            self.textes[-1].value = str(self.points)
            if self.tirs !=0:
                self.textes[-3].value= str(round((self.points/self.tirs)*100))
            else:
                self.textes[-3].value= ''
            if self.mode==1:
                if glfw.get_time()-self.debut_partie >40:
                    time.sleep(3)
                    self.points=0
                    self.tirs=0
                    self.debut_partie=glfw.get_time()
                elif self.debut_partie>0:
                    self.textes[-2].value = str(int(40-glfw.get_time()+self.debut_partie))
            elif self.mode ==2:
                self.textes[-2].value = 'infini'
                

            for obj in (self.decors+self.objs+self.textes):
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):   
                    self.update_camera(obj.program)
                obj.draw()



            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des évènements
            glfw.poll_events()

    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action           
        
       
        if key== glfw.KEY_Q and action == glfw.PRESS : # Qwerty donc appuyer sur A pour lancer le mode competitif
            if not(self.demarrage):
                self.mode= 1
                self.initialisation()

                 
        if key == glfw.KEY_B and action == glfw.PRESS: # B pour lancer le mode infini/entrainement
            if not (self.demarrage):
                self.mode =2 
                self.initialisation()



    def mouse_button_callback(self,win,button, action,mods):
        if self.demarrage :

            if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS :
                self.bool=1 
                self.coordXProj = self.mouse_x # Definit la coordonnée X de la cible
                self.coordXCible = pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[0] # Definit la coordonnée X de la cible
                self.coordZCible = pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[2] # Definit la coordonnée Z de la cible
                if self.coordXCible <0 and self.coordZCible >-12: # Si la cible est dans la zone de tir
                    ajout= 0
                    self.coordXCible= abs(200*(ajout+np.arcsin(self.coordXCible/12))) # On calcule la coordonnée X de la cible
                elif self.coordZCible <-12: 
                    ajout= np.pi
                    self.coordXCible= abs(200*(ajout+np.arcsin(self.coordXCible/12))) 
                else:
                    ajout = 3*np.pi/2
                    self.coordXCible= abs(200*(ajout+np.arccos(self.coordXCible/12)))
                

                self.y_traite = -12.006*self.y_brut +3.0126 
                if abs(self.coordXProj- self.coordXCible) <10  and abs(self.y_traite-pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[1])<0.5: 
                    self.cible_actuelle.visible = False    
                    self.points+=1
                    if self.BriqueVisible == len(self.ListeBriques)-1:
                        self.objs[1+self.BriqueVisible].visible = False
                        self.BriqueVisible = 0  
                    self.objs[1+self.BriqueVisible].visible = False
                    self.BriqueVisible +=1
                    self.objs[1+self.BriqueVisible].visible = True
                    self.cible_actuelle=self.objs[1+self.BriqueVisible]
                
                self.tirs +=1 
            
            
           
    
    def cursor_position_callback(self, win, xpos, ypos):
        if self.demarrage:
            if self.debut:
                self.modif= xpos
                self.debut=False
            xmod=(xpos-self.modif) % 1256 
            self.coordXCible = pyrr.Vector4.from_vector3(self.cible_actuelle.transformation.translation, 1)[0] 
            self.valx
            if self.mouse_x != None and self.mouse_y != None: # Si deplacement de la souris
                self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += (xmod-self.mouse_x) *0.01/2 # On tourne la caméra
                self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] +=  (xmod-self.mouse_x) *0.01/2 # On tourne le stegosaur

                if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2>=-0.4 and ypos-self.mouse_y <0: 
                    self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2



                if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y)*0.01/2<=1 and ypos-self.mouse_y >0:
                    self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += (ypos-self.mouse_y)*0.01/2

                self.y_brut = self.cam.transformation.rotation_euler[pyrr.euler.index().roll] + (ypos-self.mouse_y) *0.01/2
            self.mouse_x = xmod
            self.mouse_y = ypos




    def add_object(self, obj): 
        self.objs.append(obj)
    
    def add_textes(self,text):
        self.textes.append(text)
    
    def add_decors(self,decor):
        self.decors.append(decor)

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


    def initialisation(self):
        for i in range (len(self.ListeBriques)):
            self.objs[1+i].transformation.translation -= \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[1+i].transformation.rotation_euler), pyrr.Vector3([-self.ListeBriques[i][0], -self.ListeBriques[i][1], -self.ListeBriques[i][2]]))
        self.demarrage = True
        self.debut_partie=glfw.get_time()
        self.textes[0].visible = False
        self.textes[1].visible = False
        self.textes[2].visible = False     
        for i in range(len(self.ListeBriques)-1):
            self.objs[2+i].visible = False 
        self.cible_actuelle = self.objs[1+self.BriqueVisible]