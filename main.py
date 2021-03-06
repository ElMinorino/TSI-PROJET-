from viewerGL import ViewerGL
import glutils
import glfw
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import numpy as np
import OpenGL.GL as GL 
import pyrr
import time
import random


def main():
    viewer = ViewerGL() 

    viewer.set_camera(Camera()) 
    viewer.cam.transformation.translation.y = 2 # Deplace la camera de 2 unites en y
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy() # Definit le centre de rotation sur la translation de la caméra

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag') # Creation du programme
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag') # Id du programme pour le GUI
  

    m = Mesh.load_obj('stegosaurus.obj') 
    m.normalize() 
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1])) # Création d'une matrice d'échelle de 2x2x2
    tr = Transformation3D() 
    tr.translation.y = -np.amin(m.vertices, axis=0)[1] # Translation de l'objet pour qu'il soit au sol
    tr.translation.z = -9.5 
    tr.rotation_center.z = 0.2
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr) 
    viewer.add_object(o)

    ListeBriques=viewer.ListeBriques # Liste des briques
    for i in range (len(ListeBriques)): 
        nb_sphere = 50 
        m = Mesh()
        u = np.linspace(0, 2 * np.pi, nb_sphere) # Domaine de definition de u 
        v = np.linspace(0, np.pi, nb_sphere) # Domaine de definition de v 
        r = 0.25
        x = r * np.outer(np.cos(u), np.sin(v)) # x = r * cos(u) * sin(v)
        y = r * np.outer(np.sin(u), np.sin(v)) # y = r * sin(u) * sin(v)
        z = r * np.outer(np.ones(np.size(u)), np.cos(v)) # z = r * cos(v)
        p0, p1, p2, p3 = [x, y, z], [x, y, z], [x, y, z], [x, y, z] # Creation des 4 points de la sphere
        t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1] # Creation des 4 textures

        points = []
        for i in range(len(u)): 
            for j in range(len(v)): 
                x = r * np.outer(np.cos(u[i]), np.sin(v[j])) 
                y = r * np.outer(np.sin(u[i]), np.sin(v[j]))
                z = r * np.outer(np.ones(np.size(u[i])), np.cos(v[j]))
                sphere = [x, y, z, x/r, y/r, z/r, 1, 0, 0, 0, 0] # Creation de la sphere
                points.append(sphere) # Ajout de la sphere a la liste des points 
        points = np.array(points, np.float32) # Conversion de la liste de points en tableau numpy
        m.vertices = points # Ajout des points a la mesh
        
        t = []
        for i in range(len(u)-1): # Creation des triangles
            for j in range(len(v)-1): 
                t.append([i+j*nb_sphere, i+1+j*nb_sphere, i+(j+1)*nb_sphere]) # Triangle 1
                t.append([i+1+(j+1)*nb_sphere, i+1+j*nb_sphere, i+(j+1)*nb_sphere]) # Triangle 2
        t = np.array(t, np.uint32) 
        m.faces = t 
        texture = glutils.load_texture('wall.jpg') 
        o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D()) 
        viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [-25, 0, -25], [25, 0, -25], [25, 0, 25], [-25, 0, 25] 
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32) 
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_decors(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, 5], [15, 0, 5], [15, 10, 5], [-15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_decors(o)

    m = Mesh()
    p0, p1, p2, p3 = [15, 0, 5], [15, 0, -24], [15, 10, -24], [15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_decors(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, -24], [15, 0, -24], [15, 10, -24], [-15, 10, -24]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_decors(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, 5], [-15, 0, -24], [-15, 10, -24], [-15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c  + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_decors(o)

    # mur du haut enleve par soucis de visibilite
    # m = Mesh()
    # p0, p1, p2, p3 = [-15, 10, -24], [15, 10, -24], [15, 10, 5], [-15, 10, 5]
    # n, c = [0, 1, 0], [1, 1, 1]
    # t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    # m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    # m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    # texture = glutils.load_texture('fond.jpg')
    # o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    # viewer.add_decors(o)


    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o = Text('Bienvenue dans AIM LAB', np.array([-0.8, 0.3], np.float32), np.array([0.8, 0.6], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)#34
    o = Text('Mode competitif            Mode entrainement', np.array([-1, 0], np.float32), np.array([1,0.3], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)#34
    o = Text('Touche A                     Touche B ', np.array([-1, -0.5], np.float32), np.array([1, -0.2], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o) #35
    o= Text("x", np.array([ -0.05, 0.15], np.float32), np.array([0.05,0.05],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o) #self.objs[40]
    o= Text("Score : ", np.array([ -1, 0.9], np.float32), np.array([-0.8,1],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text("Temps restant : ", np.array([ -1, 0.8], np.float32), np.array([-0.7,0.9],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text(" secondes", np.array([ -0.6, 0.8], np.float32), np.array([-0.4,0.9],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text("Precision : ", np.array([ -1, 0.7], np.float32), np.array([-0.8,0.8],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text("", np.array([ -0.8, 0.7], np.float32), np.array([-0.7,0.8],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text("", np.array([ -0.7, 0.8], np.float32), np.array([-0.6,0.9],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)
    o= Text('0', np.array([ -0.8, 0.9], np.float32), np.array([-0.7,1],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_textes(o)

    viewer.run()
    



if __name__ == '__main__':
    main()