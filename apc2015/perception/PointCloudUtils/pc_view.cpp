#include <GL/glut.h>
#include <GLdraw/GLUTNavigationProgram.h>
#include <GLdraw/GeometryAppearance.h>
#include <GLdraw/GLLight.h>
#include <GLdraw/drawExtra.h>
#include <meshing/PointCloud.h>
using namespace std;
using namespace GLDraw;

enum {Natural,Constant,Transparent,XYZRGB,Property0};

GLColor colors [14] = {GLColor(1,1,1),
		      GLColor(1,0,0),
		      GLColor(0,1,0),
		      GLColor(0,0,1),
		      GLColor(1,0,1),
		      GLColor(1,1,0),
		      GLColor(0,1,1),
		      GLColor(0.5,0.5,0.5),
		      GLColor(1,0.5,0),
		      GLColor(0,1,0.5),
		      GLColor(0.5,0,1),
		      GLColor(0.5,1,0),
		      GLColor(0,0.5,1),
		      GLColor(0,1,0.5)
};
const static int numColors = 14;


class Viewer : public GLUTNavigationProgram
{
public:
  vector<SmartPointer<Geometry::AnyGeometry3D> > geometry;
  vector<SmartPointer<GeometryAppearance> > appearance;
  vector<GLLight> lights;
  int colorType;
  int size;

  Viewer() {
    lights.resize(1);
    lights[0].setColor(GLColor(1,1,1));
    lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
    lights[0].setColor(GLColor(1,1,1));
    colorType = Natural;
    size = 1;
  }
  virtual ~Viewer() {}

  bool LoadFile(const char* fn)
  {
    SmartPointer<Geometry::AnyGeometry3D> geom = new Geometry::AnyGeometry3D;
    if(!geom->Load(fn)) return false;
    geometry.push_back(geom);
    appearance.push_back(new GeometryAppearance());
    appearance.back()->Set(*geometry.back());
    //Colorize(appearance.size()-1,colorType);
    return true;
  }

  void Colorize(int index,int type) {
    if(type == Natural) {
      appearance[index]->Set(*geometry[index]);
    }
    else if(type == Constant) {
      GLColor c;
      if(index < numColors) 
	c = colors[index];
      else
	c.setRandom();
      for(size_t i=0;i<appearance[index]->vertexColors.size();i++)
	appearance[index]->vertexColors[i] = c;
      appearance[index]->Refresh();
    }
    else if(type == Transparent) {
      GLColor c;
      if(index < numColors) 
	c = colors[index];
      else
	c.setRandom();
      c[3] = 0.4;
      for(size_t i=0;i<appearance[index]->vertexColors.size();i++)
	appearance[index]->vertexColors[i] = c;
      appearance[index]->Refresh();
    }
    else if(type == XYZRGB) {
      Assert(geometry[index]->type == Geometry::AnyGeometry3D::PointCloud);
      Meshing::PointCloud3D& pc = geometry[index]->AsPointCloud();
      AABB3D bb;
      pc.GetAABB(bb.bmin,bb.bmax);
      if(bb.bmin.x == bb.bmax.x) bb.bmin.x -= 1.0;
      if(bb.bmin.y == bb.bmax.y) bb.bmin.y -= 1.0;
      if(bb.bmin.z == bb.bmax.z) bb.bmin.z -= 1.0;
      for(size_t i=0;i<appearance[index]->vertexColors.size();i++) {
	appearance[index]->vertexColors[i].rgba[0] = (pc.points[i].x-bb.bmin.x)/(bb.bmax.x-bb.bmin.x);
	appearance[index]->vertexColors[i].rgba[1] = (pc.points[i].y-bb.bmin.y)/(bb.bmax.y-bb.bmin.y);
	appearance[index]->vertexColors[i].rgba[2] = (pc.points[i].z-bb.bmin.z)/(bb.bmax.z-bb.bmin.z);
      }
      appearance[index]->Refresh();
    }
    else if(type >= Property0) {
      Assert(geometry[index]->type == Geometry::AnyGeometry3D::PointCloud);
      Meshing::PointCloud3D& pc = geometry[index]->AsPointCloud();
      int propertyIndex = type - Property0;
      propertyIndex = propertyIndex % pc.propertyNames.size();
      vector<Real> values(pc.properties.size());
      for(size_t i=0;i<pc.properties.size();i++) 
	values[i] = pc.properties[i][propertyIndex];
      map<Real,int> valueSet;
      for(size_t i=0;i<pc.properties.size();i++) {
	if(valueSet.count(values[i]) == 0) {
	  int k=(int)valueSet.size();
	  valueSet[values[i]] = k;
	  if(valueSet.size() > 100) break;
	}
      }
      Real vmin=values[0],vmax=values[0];
      for(size_t i=0;i<pc.properties.size();i++) 
	if(values[i] < vmin)
	  vmin = values[i];
	else if(values[i] > vmax)
	  vmax = values[i];
      if(valueSet.size() > 100) {
	for(size_t i=0;i<pc.properties.size();i++) 
	  if(vmin == vmax) //solid color
	    appearance[index]->vertexColors[i].set(1,1,1);
	  else
	    //smooth gradient
	    appearance[index]->vertexColors[i].setGray((values[i] - vmin)/(vmax-vmin));
      }
      else {
	//discrete colors
	vector<GLColor> c(valueSet.size());
	for(map<Real,int>::const_iterator i=valueSet.begin();i!=valueSet.end();i++)
	  if(i->second < numColors)
	    c[i->second] = colors[i->second];
	  else
	    c[i->second].setRandom();
	for(size_t i=0;i<appearance[index]->vertexColors.size();i++)
	    appearance[index]->vertexColors[i] = c[valueSet[values[i]]];
      }
      appearance[index]->Refresh();
    }
  }

  virtual bool Initialize() {
    viewport.n = 0.1;
    viewport.f = 100;
    viewport.setLensAngle(DtoR(30.0));
    camera.dist = 6;
    if(!geometry.empty()) {
      AABB3D bb;
      bb.minimize();
      for(size_t i=0;i<geometry.size();i++) {
	AABB3D bi = geometry[i]->GetAABB();
	bb.expand(bi.bmin);
	bb.expand(bi.bmax);
      }
      camera.tgt = (bb.bmin + bb.bmax)*0.5;
      Real size = (bb.bmax-bb.bmin).maxElement();
      camera.dist = 3.0*size;
      if(size < 0.1) viewport.n = size;
      if(size*3 > 100) viewport.f = size*3;
    }
    
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0,0,0,0);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    return GLUTNavigationProgram::Initialize();
  }

  virtual void SetWorldLights()
  {
    for(size_t i=0;i<lights.size();i++)
      lights[i].setCurrentGL((int)i);
  }
  void ClickRay(int x,int y,Ray3D& r) const
  {
    viewport.getClickSource(x,viewport.y+viewport.h-y,r.source);
    viewport.getClickVector(x,viewport.y+viewport.h-y,r.direction);
  }
  virtual void RefreshIdle() { SleepIdleCallback(0); }
  virtual void RenderWorld()
  {
    glDisable(GL_LIGHTING);
    drawCoords(0.1);
    glEnable(GL_LIGHTING);

    if(colorType == Transparent) {
      glDisable(GL_DEPTH_TEST);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    }

    for(size_t i=0;i<appearance.size();i++)
      appearance[i]->DrawGL();

    if(colorType == Transparent) {
      glEnable(GL_DEPTH_TEST);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
  }

  virtual void DoCtrlDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragPan(dx,dy);
  }
  
  virtual void DoAltDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragZoom(dx,dy);
  }

  virtual void DoShiftDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); }
  }

  virtual void Handle_Keypress(unsigned char c,int x,int y) {
    if(c == 'h') {
      printf("h: print this help message\n");
      printf("< (or ,): go to the previous color mode\n");
      printf("> (or .): go to the next color mode\n");
      printf("+ (or =): increase point size\n");
      printf("- (or _): decrease point size\n");
    }
    else if(c == ',' || c =='<') {
      colorType --;
      if(colorType < 0) colorType = Property0;
      for(size_t i=0;i<geometry.size();i++)
	Colorize(i,colorType);
      Refresh();
    }
    else if(c == '.' || c =='>') {
      colorType ++;
      for(size_t i=0;i<geometry.size();i++)
	Colorize(i,colorType);
      Refresh();
    }
    else if(c == '-' || c =='_') {
      size --;
      if(size < 1) size = 1;
      for(size_t i=0;i<geometry.size();i++) {
	appearance[i]->vertexSize = size;
	appearance[i]->Refresh();
      }
      Refresh();
    }
    else if(c == '+' || c =='=') {
      size ++;
      for(size_t i=0;i<geometry.size();i++) {
	appearance[i]->vertexSize = size;
	appearance[i]->Refresh();
      }
      Refresh();
    }
  }
};

int main(int argc,const char** argv)
{
  if(argc == 0) {
    printf("Usage: pc_view file1.pcd [file2.pcd] ...\n");
    return 0;
  }
  Viewer viewer;
  for(int i=1;i<argc;i++)
    viewer.LoadFile(argv[i]);
  viewer.Run("PCD Viewer");
}
