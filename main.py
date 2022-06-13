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
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')
  

    m = Mesh.load_obj('stegosaurus.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -9.5
    tr.rotation_center.z = 0.2
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)

    ListeBriques=viewer.ListeBriques
    for i in range (len(ListeBriques)):
        nb_sphere = 50
        m = Mesh()
        u = np.linspace(0, 2 * np.pi, nb_sphere)
        v = np.linspace(0, np.pi, nb_sphere)
        r = 0.25
        x = r * np.outer(np.cos(u), np.sin(v))
        y = r * np.outer(np.sin(u), np.sin(v))
        z = r * np.outer(np.ones(np.size(u)), np.cos(v))
        p0, p1, p2, p3 = [x, y, z], [x, y, z], [x, y, z], [x, y, z]
        n, c = [0, 2, -9.5], [0,2, -9.5]
        t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]

        points = []
        for i in range(len(u)):
            for j in range(len(v)):
                x = r * np.outer(np.cos(u[i]), np.sin(v[j]))
                y = r * np.outer(np.sin(u[i]), np.sin(v[j]))
                z = r * np.outer(np.ones(np.size(u[i])), np.cos(v[j]))
                sphere = [x, y, z, x/r, y/r, z/r, 1, 1, 1, 0, 0]
                points.append(sphere)
        points = np.array(points, np.float32)
        m.vertices = points
        
        t = []
        for i in range(len(u)-1):
            for j in range(len(v)-1):
                t.append([i+j*nb_sphere, i+1+j*nb_sphere, i+(j+1)*nb_sphere])
                t.append([i+1+(j+1)*nb_sphere, i+1+j*nb_sphere, i+(j+1)*nb_sphere])
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
    texture = glutils.load_texture('grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
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

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, 5], [15, 0, 5], [15, 10, 5], [-15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [15, 0, 5], [15, 0, -24], [15, 10, -24], [15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, -24], [15, 0, -24], [15, 10, -24], [-15, 10, -24]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 0, 5], [-15, 0, -24], [-15, 10, -24], [-15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c  + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    m = Mesh()
    p0, p1, p2, p3 = [-15, 10, -24], [15, 10, -24], [15, 10, 5], [-15, 10, 5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('fond.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)


    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o = Text('Bienvenue dans AIM LAB', np.array([-0.8, 0.3], np.float32), np.array([0.8, 0.8], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)#34

    o = Text('Appuyez sur Entree pour commencer', np.array([-0.8, -0.2], np.float32), np.array([0.8, 0.3], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o) #35

   
    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o= Text("x", np.array([ -0.05, 0.15], np.float32), np.array([0.05,0.05],np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o) #self.objs[40]
    

    viewer.run()
    



if __name__ == '__main__':
    main()