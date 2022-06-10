from viewerGL import ViewerGL
import glutils
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
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')
  

    m = Mesh.load_obj('stegosaurus.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -10
    tr.rotation_center.z = 0.2
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)
      
    text1='Bienvenue dans'
    text2='AIM LAB'
    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o = Text(text1, np.array([-0.8, 0.3], np.float32), np.array([0.8, 0.8], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text(text2, np.array([-0.5, -0.2], np.float32), np.array([0.5, 0.3], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    
    ListeXDec= [i for i in np.arange(-12.1,12,0.1)]

    ListeZDec= [i for i in np.arange(-21.5,2.5,0.1)]

    ListeBriques =[]
    for nbx in ListeXDec:
        for nbz in ListeZDec:
            if abs((nbx)**2+(nbz+9.5)**2-10**2 )<0.0001 :
                ListeBriques.append((nbz,nbx))

    
    random.shuffle(ListeBriques)
                
    for i in range (len(ListeBriques)):
        m = Mesh.load_obj('cube.obj')
        m.normalize()
        m.apply_matrix(pyrr.matrix44.create_from_scale([0.5, 0.55, 0.5, 1]))
        tr = Transformation3D()
        tr.translation.z = ListeBriques[i][0]
        tr.translation.x =ListeBriques[i][1]
        tr.translation.y = (random.random())*5+0.5
        
        texture = glutils.load_texture('wall.jpg')
        o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
        viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [-25, 0, -25], [25, 0, -25], [25, 0, 25], [-25, 0, 25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)
    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o= Text("x", np.array([ -0.05, 0.15], np.float32), np.array([0.05,0.05],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)

    m = Mesh.load_obj('cube.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([0.05, 0.05, 0.05, 1]))
    tr = Transformation3D()
    tr.translation.x = 0
    tr.translation.y = 2
    tr.translation.z = -9
    texture = glutils.load_texture('wall.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)

    viewer.run()
    
 
    


if __name__ == '__main__':
    main()