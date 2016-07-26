
	def load_model_tote_cloud_avg(self, downsample=False):
		try:
			model_cloud = np.load(self.get_tote_cloud_path())
		except:
			print 'Cannot find tote model at %s'%self.get_tote_cloud_path()
			return None
		assert len(model_cloud.shape)==3, 'Size mismatch, expecting HxWx3, current size is ' + str(model_cloud.shape)
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud
		
	def read_average_depth(self, ite=10, unit='meter'):
		camera = RemoteCamera('10.236.66.147', 30000)
		all_depth = np.zeros((480, 640, ite))
		for i in xrange(ite):
			all_depth[:,:,i] = camera.read()[1][:,:,2] # [1] to index cloud, [:,:,2] to index z axis
		camera.close()
		all_depth[all_depth==0]=np.nan
		depth_avg = np.nanmean(all_depth, axis=2)
		print 'Shape of average depth is', depth_avg.shape
		if unit in ['meter', 'm']:
			depth_avg /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			depth_avg = depth_avg
		else:
			raise Exception('Unrecognized unit '+str(unit))
		return depth_avg


	def read_cloud_avg(self, ite=10, unit='meter', Nx3_cloud=False, clean=None):
		camera = RemoteCamera('10.236.66.147', 30000)
		all_cloud = np.zeros((480, 640, 3, ite))
		for i in xrange(ite):
			all_cloud[:,:,:,i] = camera.read()[1] # [1] to index cloud
		camera.close()
		all_cloud[all_cloud==0]=np.nan
		cloud_avg = np.nanmean(all_cloud, axis=3)
		print 'Shape of average cloud is', cloud_avg.shape
		if unit in ['meter', 'm']:
			cloud_avg /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			cloud_avg = cloud_avg
		else:
			raise Exception('Unrecognized unit '+str(unit))
		if Nx3_cloud:
			cloud_avg = cloud_avg.reshape(-1, 3)
		if Nx3_cloud:
			assert clean is not None, 'clean must be a boolean when Nx3_cloud is True'
			if clean:
				keep_idxs = np.where(cloud_avg[:,2].flatten()!=np.nan)[0]
				cloud_avg = cloud_avg[keep_idxs,:]
		return cloud_avg

	def read_once_poll(self, unit='meter', Nx3_cloud=False, clean=None):
		'''
		(private) read from CameraData object
		'''
		cd = self.cd
		color = cd.color
		cloud = cd.cloud
		depth_uv = cd.depth_uv
		color_uv = cd.color_uv
		if unit in ['meter', 'm']:
			cloud /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			cloud = cloud
		else:
			raise Exception('Unrecognized unit '+str(unit))
		if Nx3_cloud:
			cloud = cloud.reshape(-1, 3)
		if Nx3_cloud:
			assert clean is not None, 'clean must be a boolean when Nx3_cloud is True'
			if clean:
				keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
				cloud = cloud[keep_idxs,:]
		return color, cloud, depth_uv, color_uv

	def save_canonical_tote_cloud_avg(self, R, t):
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(R.__class__)
		cloud = self.read_cloud_avg(ite=10, unit='meter')
		np.save(self.get_tote_cloud_path(), cloud)
		self.save_R_t(self.get_tote_viewing_camera_xform_path(), R, t)
		print 'Successfully saved averaged model for tote'

	def get_current_tote_content_cloud_fast(self, cur_camera_R, cur_camera_t, scene_cloud=None, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene in global frame
		'''
		assert fit is False, 'ICP is too time-consuming'

		if scene_cloud is None:
			scene_cloud = self.read_cloud_avg(ite=10, unit='meter')
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		scene_cloud = scene_cloud.dot(cur_camera_R.T) + cur_camera_t # transform scene cloud
		scene_depth = scene_cloud[:,:,2]
		
		model_cloud = self.load_model_tote_cloud_avg()
		assert model_cloud is not None, 'Error loading tote model'
		model_xform_R, model_xform_t = self.load_R_t(self.get_tote_viewing_camera_xform_path(), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R) + model_xform_t
		model_depth = model_cloud[:,:,2]

		depth_diff = np.abs(scene_depth - model_depth)
		depth_diff[np.isnan(depth_diff)] = 0

		print 'Maximum depth difference is:', depth_diff.max()
		plt.figure()
		plt.imshow(depth_diff)
		plt.title('Abs diff in depth')
		plt.colorbar()
		plt.figure()
		plt.imshow(depth_diff == 0)
		plt.show(block=False)

		x_select, y_select = np.where(depth_diff > threshold)
		content_cloud = scene_cloud[x_select, y_select, :]

		return content_cloud