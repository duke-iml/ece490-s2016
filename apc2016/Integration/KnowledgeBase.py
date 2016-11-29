from Group2Helper import apc

class KnowledgeBase:
    def __init__(self):
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []
        self.center_point = None
    def bin_front_center(self,bin_name):

        #center of the front face of the bin

        bmin,bmax = apc.bin_bounds[bin_name]
        # local_center = [(bmin[0]+bmax[0])*0.5, (bmin[1]+bmax[1])*0.5, bmax[2]]
        local_center = [(bmin[0]+bmax[0])*0.5, bmin[1], bmax[2]]

        # horizontal adjustment
        if bin_name == 'bin_A' or bin_name == 'bin_D' or bin_name == 'bin_G' or bin_name == 'bin_J':
            local_center = vectorops.add(local_center, [-0.0,0,0])
        elif bin_name == 'bin_B' or bin_name == 'bin_E' or bin_name == 'bin_H' or bin_name == 'bin_K':
            # local_center = vectorops.add(local_center, [-0.00,0,0])
            local_center = vectorops.add(local_center, [0.00,0,0])
        elif bin_name == 'bin_C' or bin_name == 'bin_F' or bin_name == 'bin_I' or bin_name == 'bin_L':
            local_center = vectorops.add(local_center, [-0.01,0,0])

        if bin_name == 'bin_J':
            local_center = vectorops.add(local_center, [-0.01,0,0])
        if bin_name == 'bin_K':
            local_center = vectorops.add(local_center, [-0.025,0,0])
        # in/out adjustment
        #if bin_name == 'bin_J' or bin_name == 'bin_K' or bin_name == 'bin_L':
        #    local_center = vectorops.add(local_center, [0,0,-0.003])


        world_center = se3.apply(knowledge.shelf_xform, local_center)
        return world_center
    def bin_vantage_point(self,bin_name):

        # a little in front of the front_center of the bin

        world_center = self.bin_front_center(bin_name)
        world_offset = so3.apply(knowledge.shelf_xform[0],[0,0.04,0.55])

        if bin_name == 'bin_A' or bin_name == 'bin_D' or bin_name == 'bin_G' or bin_name == 'bin_J':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])
        elif bin_name == 'bin_B' or bin_name == 'bin_E' or bin_name == 'bin_H' or bin_name == 'bin_K':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])
        elif bin_name == 'bin_C' or bin_name == 'bin_F' or bin_name == 'bin_I' or bin_name == 'bin_L':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])

        return vectorops.add(world_center,world_offset)

    def getGlobalBounds(self, bin_name):
        #center of the front face of the bin
        bmin,bmax = apc.bin_bounds[bin_name]

        globPoint1 = se3.apply(knowledge.shelf_xform_canonical, bmin)
        globPoint2 = se3.apply(knowledge.shelf_xform_canonical, bmax)

        if globPoint1[0] > globPoint2[0]:
            xmax = globPoint1[0]
            xmin = globPoint2[0]
            xmax = .8*(xmax-xmin) + xmin
        else:
            xmin = globPoint1[0]
            xmax = globPoint2[0]
            xmax = .8*(xmax-xmin) + xmin

        if globPoint1[1] > globPoint2[1]:
            ymax = globPoint1[1]
            ymin = globPoint2[1]
        else:
            ymin = globPoint1[1]
            ymax = globPoint2[1]

        if globPoint1[2] > globPoint2[2]:
            zmax = globPoint1[2]
            zmin = globPoint2[2]
        else:
            zmin = globPoint1[2]
            zmax = globPoint2[2]

        minPoint = [xmin, ymin, zmin]
        maxPoint = [xmax, ymax, zmax]

        #world_center = se3.apply(knowledge.shelf_xform, local_center)
        # print '[minPoint, maxPoint] = ', [minPoint, maxPoint]
        return [minPoint, maxPoint]

    def getBinFrontCenterTop(self, bin_name):
        #center of the front face of the bin near the top of the bin

        if bin_name == 'bin_H':
            yFactor = .7
        else:
            yFactor = .8

        minMax = apc.bin_bounds[bin_name]
        #get the min of  x,
        #get the midpoint of y
        #get close to the top of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*(1-yFactor)+minMax[1][1]*yFactor
        pointZ = minMax[1][2]

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinMidCenterTop(self, bin_name):
        #center of the front face of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the midpoint of x,
        #get the midpoint of y
        #get close to the top of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.2 + minMax[1][1]*.8
        pointZ = minMax[0][2]*.5+minMax[1][2]*.5

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinFrontCenter(self, bin_name):
        #center of the front face of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the min of  x,
        #get the midpoint of y
        #get the midpoint of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.5 + minMax[1][1]*.5
        pointZ = minMax[1][2]

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinTrueCenter(self, bin_name):
        #get the very center of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the midpoint of  x,
        #get the midpoint of y
        #get the midpoint of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.5 + minMax[1][1]*.5
        pointZ = minMax[0][2]*.5+minMax[1][2]*.5

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinWorldPosition(self, bin_name, localPoint):
        #minMax = self.getGlobalBounds(bin_name)

        bound = apc.bin_bounds[bin_name]

        if len(localPoint) ==3 and  0<= localPoint[0] <= 1 and 0<=localPoint[1] <=1 and 0<=localPoint[2] <=1:
            myPointX = bound[0][0] * localPoint[0] + (1-localPoint[0])*bound[1][0]
            myPointY = bound[0][1] * localPoint[1] + (1-localPoint[1])*bound[1][1]
            myPointZ = bound[0][2] * localPoint[2] + (1-localPoint[2])*bound[1][2]
            return se3.apply(knowledge.shelf_xform, [myPointX, myPointY, myPointZ])

        else:
            print 'Error incorrect entry format'
            return False

    def sampleBin(self, bin_name, sample=None, xPoints=3, yPoints=3, zPoints=3, startRatio = 0.1, endRatio = 0.9):
        minMax = apc.bin_bounds[bin_name]

        if sample != None:
            try:
                assert(len(sample)==3)
            except:
                print 'error in sample size - should be a list of 3 integers'
                return False
            xPoints = sample[0]
            yPoints = sample[1]
            zPoints = sample[2]

        #defaultRange = 0.1 - 0.9
        incX = (endRatio-startRatio)/xPoints
        incY = (endRatio-startRatio)/yPoints
        incZ = (endRatio-startRatio)/zPoints

        startX = startRatio

        endX = endRatio
        endY = endRatio
        endZ = endRatio

        returnPoints = []

        while startX < endX:
            startY = startRatio
            while startY < endY:
                startZ = startRatio
                while startZ < endZ:
                    xLoc = startX * minMax[0][0] + (1-startX)*minMax[1][0]
                    yLoc = startY*minMax[0][1] + (1-startY)*minMax[1][1]
                    zLoc = startZ*minMax[0][2] + (1-startZ)*minMax[1][2]
                    returnPoints.append(se3.apply(knowledge.shelf_xform, [xLoc,yLoc,zLoc]))
                    startZ = startZ + incZ
                startY = startY +incY
            startX = startX + incX

        return returnPoints

    def applyShelfXform(self, point):
        return se3.apply(knowledge.shelf_xform, point)


    def getShelfNormal(self):
        # assume z = 0
        # it's already so close to zero it doesn't affect much
        rot_matrix = knowledge.shelf_xform[0]
        # normal to the back plane of the shelf appears to point in the +z direction
        return rot_matrix[6:9]