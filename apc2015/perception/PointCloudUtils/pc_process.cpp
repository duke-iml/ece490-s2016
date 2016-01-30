#include <meshing/PointCloud.h>
#include <geometry/NeighborGraph.h>
#include <geometry/PointCloudSegmentation.h>
#include <geometry/FixedResolutionPointCloud.h>
#include <geometry/Fitting.h>
#include <math3d/AABB3D.h>
#include <math3d/Plane3D.h>
#include <utils/SmartPointer.h>
#include <utils/stringutils.h>
#include <utils/AnyValue.h>
#include <utils/permutation.h>
#include <string.h>
#include <sstream>
using namespace Meshing;
using namespace Geometry;
using namespace std;

void SegmentKMeans(PointCloud3D& pc,int k)
{
  PointCloudKMeansSegmentation seg(pc);
  bool hasAlpha = false;
  if(pc.HasProperty("rgb")) {
    pc.UnpackColorChannels();
    hasAlpha = false;
  }
  else if(pc.HasProperty("rgba")) {
    pc.UnpackColorChannels(true);
    hasAlpha = true;
  }
  //figure out the default neighborhood weights via scaling
  AABB3D bb;
  pc.GetAABB(bb.bmin,bb.bmax);
  Real size = (bb.bmax-bb.bmin).maxElement();
  Vector vmin = pc.properties[0];
  Vector vmax = pc.properties[0];
  for(size_t i=1;i<pc.properties.size();i++) {
    vmin.inplaceComponentOp(pc.properties[i],std::min<Real>);
    vmax.inplaceComponentOp(pc.properties[i],std::max<Real>);
  }
  Vector weights(pc.propertyNames.size());
  for(size_t i=0;i<pc.propertyNames.size();i++) {
    if(vmax(i) == vmin(i)) weights[i] = 0;
    else weights[i] = size / (vmax(i) - vmin(i)) * 0.2;
  }
  seg.SetPropertyWeights(weights);
  seg.Compute(k);

  if(!hasAlpha)
    pc.PackColorChannels("rgb");
  else
    pc.PackColorChannels("rgba");

  seg.AddSegmentsToPC(pc);
}

void SegmentNeighborGraph(PointCloud3D& pc,Real r)
{
  PointCloudNeighborGraphSegmentation seg(pc);
  seg.Compute(r);
  seg.AddSegmentsToPC(pc);
}

void Normals(PointCloud3D& pc,Real r)
{
  printf("Building neighbor graph %g\n",r);
  Graph::UndirectedGraph<int,int> G;
  NeighborGraph(pc,r,G);
  Vector3 viewDirection(0,0,1);
  Vector nx(pc.points.size()),ny(pc.points.size()),nz(pc.points.size());
  printf("Average neighbors per point %g\n",Real(G.NumEdges())/Real(G.NumNodes()));
  for(size_t i=0;i<pc.points.size();i++) {
    Graph::UndirectedEdgeIterator<int> e;
    vector<Vector3> pts;
    for(G.Begin(i,e);!e.end();e++) {
      Assert(0 <= e.target() && e.target() <= pc.points.size());
      pts.push_back(pc.points[e.target()]);
    }
    Plane3D p;
    if(!Geometry::FitPlane(pts,p)) 
      p.normal = -viewDirection;
    else {
      if(p.normal.dot(viewDirection) > 0)
	p.normal.inplaceNegative();
    }
    p.normal.get(nx(i),ny(i),nz(i));
  }
  pc.SetProperty("normal_x",nx);
  pc.SetProperty("normal_y",ny);
  pc.SetProperty("normal_z",nz);
}

void Set(PointCloud3D& pc,const char* property,double value)
{
  vector<Real> values(pc.points.size());
  pc.SetProperty(property,values);
}

void Merge(PointCloud3D& pc,const PointCloud3D& rhs)
{
  Assert(rhs.propertyNames == pc.propertyNames);
  pc.points.insert(pc.points.end(),rhs.points.begin(),rhs.points.end());
  pc.properties.insert(pc.properties.end(),rhs.properties.begin(),rhs.properties.end());
}

void Join(const vector<SmartPointer<PointCloud3D> >& pc,PointCloud3D& rhs)
{
  if(pc.empty()) {
    rhs.points.resize(0);
    rhs.properties.resize(0);
    return;
  }
  rhs = *pc[0];
  for(size_t i=1;i<pc.size();i++)
    Merge(rhs,*pc[i]);
}

void Subsample(const PointCloud3D& pc,Real frac,PointCloud3D& result)
{
  vector<int> indices(pc.points.size());
  RandomPermutation(indices);
  int k = (int)Ceil(frac*pc.points.size());
  result.settings = pc.settings;
  result.propertyNames = pc.propertyNames;
  result.points.resize(k);
  for(int i=0;i<k;i++) {
    result.points[i] = pc.points[indices[i]];
    result.properties[i] = pc.properties[indices[i]];
  }
}

void Resolution(const PointCloud3D& pc,Real r,PointCloud3D& result)
{
  FixedResolutionPointCloud fpc(r);
  fpc.Add(pc);
  fpc.Get(result);
  result.settings = pc.settings;
}

void Denoise(const PointCloud3D& pc,Real R,PointCloud3D& result)
{
  GridSubdivision grid(3,R);
  Vector temp(3);
  GridSubdivision::Index index;
  for(size_t i=0;i<pc.points.size();i++) {
    pc.points[i].get(temp);
    grid.PointToIndex(temp,index);
    grid.Insert(index,(void*)&pc.points[i]);
  }
  result.settings = pc.settings;
  result.propertyNames = pc.propertyNames;
  for(size_t i=0;i<pc.points.size();i++) {
    pc.points[i].get(temp);
    GridSubdivision::ObjectSet items;
    grid.BallItems(temp,R,items);
    if(items.size()>1) {  //points in neighborhood
      result.points.push_back(pc.points[i]);
      result.properties.push_back(pc.properties[i]);
    }
  }
}

bool Segment(PointCloud3D& pc,const char* method,const char* param)
{
  if(0==strcmp(method,"kmeans")) {
    int k;
    if(!LexicalCast(param,k)) return false;
    SegmentKMeans(pc,k);
    return true;
  }
  else if(0==strcmp(method,"neighborgraph")) {
    Real r;
    if(!LexicalCast(param,r)) return false;
    SegmentNeighborGraph(pc,r);
    return true;
  }
  else {
    printf("Invalid method %s\n",method);
    printf("Available methods: kmeans, neighborgraph\n");
  }
  return false;
}

bool Normals(PointCloud3D& pc,const char* radius)
{
  double dradius;
  if(!LexicalCast(radius,dradius)) return false;
  Normals(pc,dradius);
  return true;
}

bool Set(PointCloud3D& pc,const char* property,const char* value)
{
  double dvalue;
  if(!LexicalCast(value,dvalue)) return false;
  Set(pc,property,dvalue);
  return true;
}

bool Merge(PointCloud3D& pc,const char* pcdfile)
{
  PointCloud3D rhs;
  if(!rhs.LoadPCL(pcdfile)) return false;
  Merge(pc,rhs);
  return true;
}

bool Split(const PointCloud3D& pc,const char* property,vector<SmartPointer<PointCloud3D> >& results)
{
  vector<Real> values;
  if(!pc.GetProperty(property,values)) return false;
  map<Real,vector<int> > segments;
  for(size_t i=0;i<values.size();i++)
    segments[values[i]].push_back(i);
  if(segments.size() > 1000) {
    printf("Warning, split would create over 1000 segments... continue?\n");
    printf("(y/n) > "); fflush(stdout);
    int c=getchar();
    if(c != 'y') return false;
  }
  results.resize(segments.size());
  int k=0;
  for(map<Real,vector<int> >::const_iterator i=segments.begin();i!=segments.end();i++,k++) {
    results[k] = new PointCloud3D;
    results[k]->settings = pc.settings;
    results[k]->propertyNames = pc.propertyNames;
    results[k]->points.resize(i->second.size());
    results[k]->properties.resize(i->second.size());
    for(size_t j=0;j<i->second.size();j++) {
      results[k]->points[j] = pc.points[i->second[j]];
      results[k]->properties[j] = pc.properties[i->second[j]];
    }
  }
  return true;
}

bool Subsample(const PointCloud3D& pc,const char* frac,PointCloud3D& result)
{
  Real dfrac;
  if(!LexicalCast(frac,dfrac)) return false;
  Subsample(pc,dfrac,result);
  return true;
}

bool Resolution(const PointCloud3D& pc,const char* radius,PointCloud3D& result)
{
  Real dradius;
  if(!LexicalCast(radius,dradius)) return false;
  Resolution(pc,dradius,result);
  return true;
}

bool Denoise(const PointCloud3D& pc,const char* radius,PointCloud3D& result)
{
  Real dradius;
  if(!LexicalCast(radius,dradius)) return false;
  Denoise(pc,dradius,result);
  return true;
}

bool Save(const vector<SmartPointer<PointCloud3D> >& clouds,const string& fn)
{
  if(clouds.size()==1) {
    printf("Saving result to %s\n",fn.c_str());
    return clouds[0]->SavePCL(fn.c_str());
  }
  string base = fn;
  StripExtension(base);
  for(size_t i=0;i<clouds.size();i++) {
    stringstream ss;
    ss << base << "_"<<i<<".pcd";
    printf("Saving result %d to %s\n",i,ss.str().c_str());
    if(!clouds[i]->SavePCL(ss.str().c_str())) {
      printf("  Error saving!\n");
      return false;
    }
  }
  return true;
}

int main(int argc,const char** argv)
{
  if(argc < 3) {
    printf("USAGE: pc_process file.pcd OPERATION [ARGS] [out.pcd]\n");
    printf("Command-line utility for processing point clouds\n");
    printf("Operations:\n");
    printf(" - segment METHOD [ARGS]: segment using the method METHOD\n");
    printf("   which can be 'kmeans K', 'neighborgraph R' where K and R\n");
    printf("   are parameters.\n");
    printf(" - normals R: estimates normals using a grid with resolution\n");
    printf("   R and plane fitting.\n");
    printf(" - split PROPERTY: splits the PC on the given property into\n");
    printf("   multiple point clouds.\n");
    printf(" - join: joins split files.\n");
    printf(" - merge FILE: merges with the given file.\n");
    printf(" - set PROPERTY VALUE: sets all points to have value VALUE\n");
    printf("   under property PROPERTY.\n");
    printf(" - resolution R: collapses point cloud to resolution r.\n");
    printf(" - subsample FRAC: subsamples by the given fraction.\n");
    printf(" - denoise R: denoise by deleting any outliers where no other\n");
    printf("   points lie within radius R.\n");
    return 0;
  }
  const char* infile = argv[1];
  const char* op = argv[2];
  vector<SmartPointer<PointCloud3D> > cur(1);
  cur[0] = new PointCloud3D;
  if(!cur[0]->LoadPCL(infile)) {
    printf("Error reading .pcd file %s\n",infile);
    return -1;
  }
  cur[0]->SetXYZAsProperties(false);
  printf("Loaded point cloud with %d points, properties: ",cur[0]->points.size());
  for(size_t i=0;i<cur[0]->propertyNames.size();i++)
    printf("%s, ",cur[0]->propertyNames[i].c_str());
  printf("\n");

  int i=2;
  vector<string> ops;
  vector<SmartPointer<PointCloud3D> > next;
  while(i < argc) {
    ops.push_back(op);
    if(0==strcmp(op,"segment")) {
      if(i+2 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = cur[c];
	  if(!Segment(*cur[c],argv[i+1],argv[i+2])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 2;
      }
      else {
	printf("Not enough arguments to segment\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"set")) {
      if(i+2 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = cur[c];
	  if(!Set(*cur[c],argv[i+1],argv[i+2])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 2;
      }
      else {
	printf("Not enough arguments to segment\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"subsample")) {
      if(i+1 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = new PointCloud3D;
	  if(!Subsample(*cur[c],argv[i+1],*next[c])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to subsample\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"normals")) {
      if(i+1 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = cur[c];
	  if(!Normals(*cur[c],argv[i+1])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to normals\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"denoise")) {
      if(i+1 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = new PointCloud3D;
	  if(!Denoise(*cur[c],argv[i+1],*next[c])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to denoise\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"resolution")) {
      if(i+1 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = new PointCloud3D;
	  if(!Resolution(*cur[c],argv[i+1],*next[c])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to merge\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"merge")) {
      if(i+1 < argc) {
	next.resize(cur.size());
	for(size_t c=0;c<cur.size();c++) {
	  next[c] = cur[c];
	  if(!Merge(*cur[c],argv[i+1])) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to merge\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"split")) {
      if(i+1 < argc) {
	next.resize(0);
	for(size_t c=0;c<cur.size();c++) {
	  vector<SmartPointer<PointCloud3D> > nextc;
	  if(!Split(*cur[c],argv[i+1],nextc)) {
	    printf("Error performing %s\n",op);
	    return -1;
	  }
	  next.insert(next.end(),nextc.begin(),nextc.end());
	}
	i += 1;
      }
      else {
	printf("Not enough arguments to split\n");
	return -1;
      }
    }
    else if(0==strcmp(op,"join")) {
      if(i+1 < argc) {
	next.resize(1);
	next[0] = new PointCloud3D;
	Join(cur,*next[0]);
	i += 1;
      }
      else {
	printf("Not enough arguments to split\n");
	return -1;
      }
    }
    else {
      printf("Invalid operation %s\n",op);
      return -1;
    }
    //debug information
    if(next.size()==1) {
      printf("Processed to point cloud with %d points\n",next[0]->points.size());
    }
    else {
      printf("Processed to %d point clouds sizes: ",next.size());
      for(size_t k=0;k<next.size();k++)
	printf("%d, ",next[k]->points.size());
      printf("\n");
    }
    //test whether we need to save
    if(i+1 == argc) {
      //save to default output file
      string default_fn = infile;
      StripExtension(default_fn);
      for(size_t o=0;o<ops.size();o++)
	default_fn += string("_") + ops[o];
      default_fn += ".pcd";
      Save(next,default_fn);
      return 0;
    }
    else if(i+2 == argc) {
      //save to an output file
      Save(next,argv[i+1]);
      return 0;
    }
    else {
      //multi-op call, go to the next operation
      swap(cur,next);
      if(i+1 < argc)
	op = argv[i+1];
      i++;
    }
  }
  return 0;
}
