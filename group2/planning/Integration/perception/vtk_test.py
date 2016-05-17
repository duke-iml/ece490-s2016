
import vtk
import numpy as np

class VtkPointCloud:

	def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e6):
		self.maxNumPoints = maxNumPoints
		self.vtkPolyData = vtk.vtkPolyData()
		self.clearPoints()
		mapper = vtk.vtkPolyDataMapper()
		mapper.SetInputData(self.vtkPolyData)
		mapper.SetColorModeToDefault()
		mapper.SetScalarRange(zMin, zMax)
		mapper.SetScalarVisibility(1)
		self.vtkActor = vtk.vtkActor()
		self.vtkActor.SetMapper(mapper)

	def addPoint(self, point):
		if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
			pointId = self.vtkPoints.InsertNextPoint(point[:])
			self.vtkDepth.InsertNextValue(point[2])
			self.vtkCells.InsertNextCell(1)
			self.vtkCells.InsertCellPoint(pointId)
		else:
			r = random.randint(0, self.maxNumPoints)
			self.vtkPoints.SetPoint(r, point[:])
		self.vtkCells.Modified()
		self.vtkPoints.Modified()
		self.vtkDepth.Modified()

	def clearPoints(self):
		self.vtkPoints = vtk.vtkPoints()
		self.vtkCells = vtk.vtkCellArray()
		self.vtkDepth = vtk.vtkDoubleArray()
		self.vtkDepth.SetName('DepthArray')
		self.vtkPolyData.SetPoints(self.vtkPoints)
		self.vtkPolyData.SetVerts(self.vtkCells)
		self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
		self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')


pointCloud = VtkPointCloud()

pts = np.load('cached_bin_cloud/binA.npy').tolist()
pts_perturbed = np.load('cached_bin_cloud/binA_perturbed.npy').tolist()

for pt in pts:
	pointCloud.addPoint(pt)

for pt in pts_perturbed:
	pointCloud.addPoint(pt)

# Renderer
renderer = vtk.vtkRenderer()
renderer.AddActor(pointCloud.vtkActor)
renderer.SetBackground(.2, .3, .4)
renderer.ResetCamera()

# Render Window
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)

# Interactor
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)

# Begin Interaction
renderWindow.Render()
renderWindowInteractor.Start()
