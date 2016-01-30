import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')
logging.getLogger('OpenGL').setLevel(99)

# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from klampt.robotsim import WorldModel
from klampt import se3, gldraw
from OpenGL.GL import glEnable, glDisable, glBlendFunc, glMaterialfv, glColor3f, glMatrixMode, glLoadIdentity, glMultMatrixf, glOrtho, glRasterPos3f
from OpenGL.GL import GL_BLEND, GL_SRC_ALPHA, GL_FRONT_AND_BACK, GL_ONE_MINUS_SRC_ALPHA, GL_AMBIENT_AND_DIFFUSE
from OpenGL.GL import GL_LIGHTING, GL_PROJECTION, GL_MODELVIEW
from OpenGL.GLU import gluPerspective, gluProject
from OpenGL.GLUT import glutLeaveMainLoop, glutPostRedisplay, glutPositionWindow, GLUT_BITMAP_HELVETICA_12

from integration.visualization import WorldViewer

world = WorldModel()
# shelf = world.loadRigidObject('klampt_models/north_shelf/shelf_with_bins.obj')
shelf = world.loadRigidObject('klampt_models/kiva_pod/model.obj')

bin_vantage_points = json.load(open('kb/bin_vantage_points.json'))
vantage_point_xforms = json.load(open('kb/vantage_point_xforms.json'))

# trabsform vantage points into world coordinates
for k in vantage_point_xforms.keys():
    vantage_point_xforms[k] = se3.mul(shelf.getTransform(), vantage_point_xforms[k])

class Viewer(WorldViewer):
    def display_screen(self):
        glDisable(GL_LIGHTING)

        glColor3f(1, 1, 1)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective (self.fov,float(self.width)/float(self.height),self.clippingplanes[0],self.clippingplanes[1])

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # View transformation
        mat = se3.homogeneous(self.camera.matrix())
        cols = zip(*mat)
        pack = sum((list(c) for c in cols),[])
        glMultMatrixf(pack)

        labels = dict([ (k, T[1]) for (k, T) in vantage_point_xforms.items() ])
        for (k, v) in labels.items():
            try:
                labels[k] = gluProject(*v)
            except:
                labels[k] = None

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,self.width,self.height,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        for (s, p) in labels.items():
            if p:
                glRasterPos3f(p[0], self.height - p[1], p[2])
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12, s)

viewer = Viewer(world, vantage_point_xforms.values())
viewer.run()
