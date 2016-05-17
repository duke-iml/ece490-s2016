#!/usr/bin/python

from OpenGL import GL
from OpenGL.GL import glViewport, glClear, glDrawPixels, glClearColor, glClearDepth, glMatrixMode, glLoadIdentity, glRasterPos2f, glDrawPixels, glPixelZoom
from PySide.QtGui import QSizePolicy
from PySide.QtOpenGL import QGLWidget

import numpy

opengl_pixel_formats = {3: GL.GL_RGB,
                        4: GL.GL_RGBA,
                        1: GL.GL_LUMINANCE,
                        2: GL.GL_LUMINANCE_ALPHA}
                        
opengl_data_types = {	numpy.uint8:	GL.GL_UNSIGNED_BYTE,
                        numpy.int8:		GL.GL_BYTE,
                        numpy.uint16:	GL.GL_UNSIGNED_SHORT,
                        numpy.int16:	GL.GL_SHORT,
                        numpy.uint32:	GL.GL_UNSIGNED_INT,
                        numpy.int32:	GL.GL_INT,
                        numpy.float32: 	GL.GL_FLOAT}

class NumpyWidget(QGLWidget):
    def __init__(self, **kwargs):
        QGLWidget.__init__(self, **kwargs)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

        self.opengl_resize = kwargs.get('resize', True)
        self.opengl_keep_aspect = kwargs.get('keep_aspect', True)
        self.opengl_data = None

    def paintGL(self):    
        if self.opengl_data is None:
            glClear(GL.GL_COLOR_BUFFER_BIT)
        else:
            if self.opengl_resize and self.opengl_keep_aspect:
                x_scale = self.width() / float(self.opengl_width)
                y_scale = self.height() / float(self.opengl_height)

                scale = min(x_scale, y_scale)

                if x_scale > y_scale:
                    glRasterPos2f(-self.opengl_width * scale / self.width(), -1)
                else:
                    glRasterPos2f(-1, -self.opengl_height * scale / self.height())
            else:
                glRasterPos2f(-1, -1)

            glDrawPixels(self.opengl_width, self.opengl_height, self.opengl_format, self.opengl_dtype, self.opengl_data)
        
    def resizeGL(self, w, h):        
        glViewport(0, 0, w, h)
        if self.opengl_resize and self.opengl_data is not None:
            x_scale = self.width() / float(self.opengl_width)
            y_scale = self.height() / float(self.opengl_height)

            if self.opengl_keep_aspect:
                scale = min(x_scale, y_scale)
                glPixelZoom(scale, scale)
            else:
                glPizelZoom(x_scale, y_scale)
    
    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClearDepth(1.0)

        glMatrixMode(GL.GL_PROJECTION)
        glLoadIdentity()
    
    def update_frame(self, numpy_image, format=None, datatype=None):
        self.opengl_data = numpy.flipud(numpy_image).tostring()
        (self.opengl_height, self.opengl_width) = numpy_image.shape[0:2]

        if format:
            self.opengl_format = format
        elif len(numpy_image.shape) == 2:
            self.opengl_format = GL.GL_LUMINANCE
        else:
            # choose pixel format based on image depth
            self.opengl_format = opengl_pixel_formats[numpy_image.shape[2]]
        
        if datatype:
            self.opengl_dtype = datatype
        else:
            self.opengl_dtype = opengl_data_types[numpy_image.dtype.type]
        
        #self.setMaximumSize(self.opengl_width, self.opengl_height)
        self.updateGL()
        
if __name__ == '__main__':
    from PySide.QtGui import QApplication
    import sys

    app = QApplication(sys.argv)
    nw = NumpyWidget()
    nw.show()
    
    nw.update_frame(numpy.random.uniform(size=(100,100)))
    
    app.exec_()
    