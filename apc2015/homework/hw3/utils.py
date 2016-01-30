import json


def get_reconstructed_model(input_file,colors=False):
    """
    Read the jason file containing the reconstructed model and return the depth
    map as a list with format: [[X1,Y1,Z1],[X2,Y2,Z2],...]

    If colors = True, then a dictionary with elements
    {'positions':plist,'colors':clist} is returned.  In this case, plist
    is the same as before, while clist is a list of [r,g,b,a] values coloring
    each point.  Each value in r,g,b,a is an integer in the range [0,255].
    If colors are not available, clist is an empty list
    """
    json_data = open(input_file)
    data = json.load(json_data)
    json_data.close()
    vertices = data['vertices'][0]['values']
    # The coordinates of the points are
    px = [vertices[i] for i in range(0,len(vertices),3)]
    py = [vertices[i] for i in range(1,len(vertices),3)]
    pz = [vertices[i] for i in range(2,len(vertices),3)]
    # Convert to the output format
    Pout = []
    for i in range(len(px)):
        Pout.append([px[i],py[i],pz[i]],)
    if not colors:
        return Pout
    Cout = []
    for element in data['vertices']:
        if element['name'] == 'color_buffer':
            colors = element['values']
            r = [colors[i] for i in range(0,len(colors),4)]
            g = [colors[i] for i in range(1,len(colors),4)]
            b = [colors[i] for i in range(2,len(colors),4)]
            a = [colors[i] for i in range(3,len(colors),4)]
            for i in range(len(r)):
                Cout.append([r[i],g[i],b[i],a[i]])
    return {'positions':Pout,'colors':Cout}


def get_raw_depth(input_file):
    """
    Read the jason file containing the raw depth and return the depth map a 2D
    list where each row and column indicate the position (in pixels) associated
    with the corresponding depth

    """
    json_data = open(input_file)
    data = json.load(json_data)
    json_data.close()
    depth = data['depth']
    return depth

def get_camera_parameters(input_file):
    """
    Get the intrinsic camera parameters (K).
    """
    json_data = open(input_file)
    data = json.load(json_data)
    json_data.close()
    K = data['K']
    return K




# -------------------------------------------------------------------------
#     Open GL Class
# -------------------------------------------------------------------------


# For opengl
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class OpenGLPlot(object):
    """
    Class to plot the cloud points in OpenGL
    """
    def __init__(self, Pobj, Pobj2, width=640, height=480):
        self.width = width
        self.height = height
        self.Pobj = Pobj
        self.Pobj2 = Pobj2
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(width, height)
        glutInitWindowPosition(0, 0)
        glutCreateWindow("Points")       
        glutDisplayFunc(self.display_function)    # Callback function
        glutKeyboardFunc(self.keyboard_function)  # Function for keyboard
        self.initializeGL()

        #setup the OpenGL display lists
        self.Pobj_glList = glGenLists(1)
        glNewList(self.Pobj_glList,GL_COMPILE)
        if isinstance(self.Pobj,dict):
            assert 'positions' in self.Pobj and 'colors' in self.Pobj,"Colored object must have 'positions' and 'colors' lists"
            glBegin(GL_POINTS)
            points = self.Pobj['positions']
            colors = self.Pobj['colors']
            for i in range(len(points)):
                glColor4f(colors[i][0]/255.0,colors[i][1]/255.0,colors[i][2]/255.0,colors[i][3]/255.0)
                glVertex3f(points[i][0], points[i][1], points[i][2])
            glEnd()                            
        else:
            glBegin(GL_POINTS)                 
            for i in range(len(self.Pobj)):
                glColor3f(1.0, self.Pobj[i][2]*1.5, self.Pobj[i][0]*1.5)
                glVertex3f(self.Pobj[i][0], self.Pobj[i][1], self.Pobj[i][2])
            glEnd()                            
        glEndList()
        self.Pobj2_glList = glGenLists(1)
        glNewList(self.Pobj2_glList,GL_COMPILE)
        glBegin(GL_POINTS)                 
        for i in range(len(self.Pobj2)):
            glColor3f(0.0, self.Pobj2[i][2]*1.5, self.Pobj2[i][0]*1.5)
            glVertex3f(self.Pobj2[i][0], self.Pobj2[i][1], self.Pobj2[i][2])
        glEnd()                            
        glEndList()

    def initialize_main_loop(self):
        glutMainLoop()

        
    def initializeGL(self):
        """Set all the initial parameters
        """
        glClearColor (0.0, 0.0, 0.0, 0.0) # Clear the background color to black
        glClearDepth(1.0)		  # Enable clearing of the depth buffer
        glDepthFunc(GL_LESS)		  # type of depth test to do
        glEnable(GL_DEPTH_TEST)		  # Enable depth testing
        glShadeModel(GL_SMOOTH)		  # Enable smooth color shading
        # Object to handle the mouse
        self.mouseInteractor = MouseInteractor(0.01, 1.0)
        self.mouseInteractor.registerCallbacks()

    def keyboard_function(self, *args):
        """If esc is pressed, end
        """
        if args[0] == '\033':
	    sys.exit()

    def display_function(self):
        """Function that is called all the time
        """
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # xSize, ySize = glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT)
        # gluPerspective(60, float(xSize) / float(ySize), 0.1, 50)
        gluPerspective(60, float(self.width)/float(self.height), 0.01, 10)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -1.0)
        self.mouseInteractor.applyTransformation()
        self.draw_cloud()
        glutSwapBuffers()

    def draw_cloud(self):
        """
        Draw the cloud points
        """
        factor = 0.1
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)   
        glVertex3f(1.0, 0.0, 0.0)
        glEnd()
        glColor3f(0.0, 1.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)   
        glVertex3f(0.0, 1.0, 0.0)
        glEnd()
        glColor3f(0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)   
        glVertex3f(0.0, 0.0, 1.0)
        glEnd()
        
        glCallList(self.Pobj_glList)
        glCallList(self.Pobj2_glList)
        
        
        
    def update_point(self, Pin):
        self.Pobj = Pin
        #update the OpenGL display list
        glDeleteLists(self.Pobj_glList,1)
        self.Pobj_glList = glGenLists(1)
        glNewList(self.Pobj_glList,GL_COMPILE)
        if isinstance(self.Pobj,dict):
            assert 'positions' in self.Pobj and 'colors' in self.Pobj,"Colored object must have 'positions' and 'colors' lists"
            glBegin(GL_POINTS)
            points = self.Pobj['positions']
            colors = self.Pobj['colors']
            for i in range(len(points)):
                glColor4i(colors[i][0]/255.0,colors[i][1]/255.0,colors[i][2]/255.0,colors[i][3]/255.0)
                glVertex3f(points[i][0], points[i][1], points[i][2])
            glEnd()                            
        else:
            glBegin(GL_POINTS)                 
            for i in range(len(self.Pobj)):
                glColor3f(1.0, self.Pobj[i][2]*1.5, self.Pobj[i][0]*1.5)
                glVertex3f(self.Pobj[i][0], self.Pobj[i][1], self.Pobj[i][2])
            glEnd()                            
        glEndList()

# -------------------------------------------------------------------------
#     Classes to handle the mouse in OpenGL
# -------------------------------------------------------------------------


class TransformationMatrix (object):
    """
    Matrix that represents a rigid transformation
    """

    def __init__(self):
        self.currentMatrix_ = None
        self.reset()

    def reset(self):
        """
        Initialize internal matrix with the identity
        """
        glPushMatrix()
        glLoadIdentity()
        self.currentMatrix_ = glGetFloatv(GL_MODELVIEW_MATRIX)
        glPopMatrix()

    def addTranslation(self, tx, ty, tz):
        """
        Apply a translation to the internal matrix
        """
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(tx, ty, tz)
        glMultMatrixf(self.currentMatrix_)
        self.currentMatrix_ = glGetFloatv(GL_MODELVIEW_MATRIX)
        glPopMatrix()

    def addRotation(self, ang, rx, ry, rz):
        """
        Apply a rotation to the internal matrix
        """
        glPushMatrix()
        glLoadIdentity()
        glRotatef(ang, rx, ry, rz)
        glMultMatrixf(self.currentMatrix_)
        self.currentMatrix_ = glGetFloatv(GL_MODELVIEW_MATRIX)
        glPopMatrix()

    def getCurrentMatrix(self):
        return self.currentMatrix_



class MouseInteractor (object):
    """
    Connect mouse motion with transformation matrix
    """

    def __init__(self, translationScale=0.1, rotationScale=.2):
        self.scalingFactorRotation    = rotationScale
        self.scalingFactorTranslation = translationScale
        self.rotationMatrix           = TransformationMatrix()
        self.translationMatrix        = TransformationMatrix()
        self.mouseButtonPressed       = None
        self.oldMousePos              = [0, 0]

    def mouseButton(self, button, mode, x, y):
        """
        Callback function for mouse button
        """
        if mode == GLUT_DOWN:
            self.mouseButtonPressed = button
        else:
            self.mouseButtonPressed = None
        self.oldMousePos[0], self.oldMousePos[1] = x, y
        glutPostRedisplay()

        
    def mouseMotion(self, x, y):
        """
        Callback function for mouse motion. Depending on the button, the
        displacement of the mouse pointer is converted to a translation vector
        or a rotation matrix.
        """
        deltaX = x - self.oldMousePos[0]
        deltaY = y - self.oldMousePos[1]
        if self.mouseButtonPressed == GLUT_RIGHT_BUTTON:
            tX = deltaX * self.scalingFactorTranslation
            tY = deltaY * self.scalingFactorTranslation
            self.translationMatrix.addTranslation(tX, -tY, 0)
        elif self.mouseButtonPressed == GLUT_LEFT_BUTTON:
            rY = deltaX * self.scalingFactorRotation
            self.rotationMatrix.addRotation(rY, 0, 1, 0)
            rX = deltaY * self.scalingFactorRotation
            self.rotationMatrix.addRotation(rX, 1, 0, 0)
        else:
            tZ = deltaY * self.scalingFactorTranslation
            self.translationMatrix.addTranslation(0, 0, tZ)
        self.oldMousePos[0], self.oldMousePos[1] = x, y
        glutPostRedisplay()

        
    def applyTransformation(self):
        """
        Add the current transformation matrix to the current OpenGL transformation
        matrix (matrix multiplication)
        """
        glMultMatrixf(self.translationMatrix.getCurrentMatrix())
        glMultMatrixf(self.rotationMatrix.getCurrentMatrix())

        
    def registerCallbacks(self):
        """
        Initialize glut callback functions
        """
        glutMouseFunc(self.mouseButton)
        glutMotionFunc(self.mouseMotion)


        
# -------------------------------------------------------------------------
#     Plot with matplotlibs, if available
# -------------------------------------------------------------------------

try:
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
except ImportError:
    matplotavailable = False
    
def matplot_points(P1, P2):
    fig = plt.figure()
    ax  = fig.gca(projection='3d')
    ax.plot(zip(*P1)[0], zip(*P1)[1], zip(*P1)[2], '.')
    ax.plot(zip(*P2)[0], zip(*P2)[1], zip(*P2)[2], '.')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.show()


def matplot_points3(P1, P2, P3):
    fig = plt.figure()
    ax  = fig.gca(projection='3d')
    ax.plot(zip(*P1)[0], zip(*P1)[1], zip(*P1)[2], '.')
    ax.plot(zip(*P2)[0], zip(*P2)[1], zip(*P2)[2], '.')
    ax.plot(zip(*P3)[0], zip(*P3)[1], zip(*P3)[2], '.')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.show()

