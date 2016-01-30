#!/usr/bin/python
from klampt import *
from klampt import so3,se3,vectorops
from klampt import resource
import apc
import os
import sys
import json

objectname = 'champion_copper_plus_spark_plug'
grippername = 'reflex'
multi = 0

#grippername = 'rethink_electric_gripper'

if len(sys.argv) < 2:
    print "USAGE: grasp_editor.py gripper object [grasp_name_or_index]"
    print "  Third argument may also be used to interpolate translation between two grasps"
    print "  Value of the third argument is the number of interpolated grasps, set to 0 for old behavior"
    print "  Fourth argument may also be 'list' to show the available grasps"
    exit(0)

if len(sys.argv) > 3:
    multi = int(sys.argv[3])
if len(sys.argv) > 2:
    grippername = sys.argv[1]
    objectname = sys.argv[2]
else:
    objectname = sys.argv[1]
    print "Adding new grasp for",grippername,"by default"

#load gripper modules
sys.path.append('../klampt_models')
import rethink_electric_gripper,reflex_col

grippermodules = {'reflex':reflex_col,
                  'rethink_electric_gripper':rethink_electric_gripper,
                  }
grippermodule = grippermodules[grippername]

world = WorldModel()
world.readFile(os.path.join('../klampt_models/',grippermodule.klampt_model_name))
robot = world.robot(0)

#load the items and spawn one in the world
apc.load_item(objectname,gripper=grippername)
item = apc.item_info[objectname]
item.geometry = apc.load_item_geometry(item)
itemposed = apc.ItemInBin(item,'blah')
itemposed.xform = se3.identity()
apc.spawn_item_in_world(itemposed,world)

#set the directory
print "Resouce directory is",os.path.join('../resources/',os.path.splitext(grippermodule.klampt_model_name)[0])
resource.setDirectory(os.path.join('../resources/',os.path.splitext(grippermodule.klampt_model_name)[0]))

#edit the default configurations
default_open_config = resource.get('default_open.config',
                                   description='Default hand open configuration',
                                   doedit='auto',
                                   world=world)
default_closed_config = resource.get('default_closed.config',
                                     description='Default hand closed configuration',
                                     doedit='auto',
                                     world=world)


def editNewGrasp(item,object,name=None,openClose=1):
    global world,robot
    global default_open_config,default_close_config
    #get the object transform
    xform = resource.get(name=None,
                         type='RigidTransform',
                         default=object.getTransform(),
                         description="Transform of "+item.name,
                         world=world,frame=object)
    if xform==None: return None
    #set the object in the world 
    object.setTransform(*xform)

    if openClose:
        #get the hand open configuration
        open_config = resource.get(name=None,type='Config',default=default_open_config,description="Hand open configuration",world=world)
        if open_config==None: return None
        closed_config = resource.get(name=None,type='Config',default=open_config,description="Hand closed configuration",world=world)
        if closed_config==None: return None

    #return grasp
    grasp = apc.ItemGrasp()
    grasp.grasp_xform = se3.inv(xform)

    if openClose:
        grasp.gripper_close_command = grippermodule.configToCommand(closed_config)
        grasp.gripper_open_command = grippermodule.configToCommand(open_config)
    if name==None:
        #default: just number it
        grasp.name = str(len(item.grasps))
    else:
        grasp.name = name
    return grasp


def editExistingGrasp(item,object,gindex):
    global world,robot
    grasp = None
    if isinstance(gindex,int):
        grasp = item.grasps[gindex]
    else:
        for g in item.grasps:
            if g.name == gindex:
                grasp = g
                break
        if grasp == None:
            print "Grasp",gindex,"does not exist, creating new grasp"
            raise RuntimeError("Invalid grasp")
    #get the object transform
    object.setTransform(*se3.inv(grasp.grasp_xform))
    open_config = grippermodule.commandToConfig(grasp.gripper_open_command)
    closed_config = grippermodule.commandToConfig(grasp.gripper_close_command)
    robot.setConfig(closed_config)
    xform = resource.get(name=None,type='RigidTransform',default=object.getTransform(),description="Transform of "+item.name,world=world,frame=object)
    if xform==None: return None
    #set the object in the world 
    object.setTransform(*xform)
    #get the hand open configuration
    open_config = resource.get(name=None,type='Config',default=open_config,description="Hand open configuration",world=world)
    if open_config==None: return None
    closed_config = resource.get(name=None,type='Config',default=closed_config,description="Hand closed configuration",world=world)
    if closed_config==None: return None
    #return grasp
    grasp.grasp_xform = se3.inv(xform)
    grasp.gripper_close_command = grippermodule.configToCommand(closed_config)
    grasp.gripper_open_command = grippermodule.configToCommand(open_config)
    return grasp

editedGrasp = None
if len(sys.argv) > 4:
    gindex = sys.argv[4]
    if gindex=='list':
        print 
        print "List of available grasps:"
        for g in item.grasps:
            print "   ",g.name
        print
        exit(0)
    try:
        editedGrasp = editExistingGrasp(item,world.rigidObject(0),gindex)
    except Exception:
        editedGrasp = editNewGrasp(item,world.rigidObject(0),gindex)
else:
    if multi:
        editedGrasp_a = editNewGrasp(item,world.rigidObject(0))
        editedGrasp_b = editNewGrasp(item,world.rigidObject(0),openClose=0)
        if editedGrasp_a == None or editedGrasp_b == None:
            print "Edit canceled, not writing to disk"
            exit(0)
    else:
        editedGrasp = editNewGrasp(item,world.rigidObject(0))
        if editedGrasp == None:
            print "Edit canceled, not writing to disk"
            exit(0)

if multi:
    base_name = editedGrasp_a.name
    grasps = []
    for i in range(0, multi+2):
        alpha = i * (1.0 / (float(multi) + 1.0))

        grasps.append(apc.ItemGrasp())
        # linearly interpolate between two grasp translations (just use rotation of grasp a)
        translation_x = (1 - alpha) * editedGrasp_a.grasp_xform[1][0] + alpha * editedGrasp_b.grasp_xform[1][0]
        translation_y = (1 - alpha) * editedGrasp_a.grasp_xform[1][1] + alpha * editedGrasp_b.grasp_xform[1][1]
        translation_z = (1 - alpha) * editedGrasp_a.grasp_xform[1][2] + alpha * editedGrasp_b.grasp_xform[1][2]
        translation = [translation_x, translation_y, translation_z]
        grasps[i].grasp_xform = (editedGrasp_a.grasp_xform[0], translation)

        grasps[i].gripper_close_command = editedGrasp_a.gripper_close_command
        grasps[i].gripper_open_command = editedGrasp_a.gripper_close_command
        grasps[i].name = base_name + '_interp_' + str(i)

        #save grasp to disk
        graspFile = os.path.join(apc.item_model_path,objectname,"grasps",grippername,grasps[i].name+".json")
        print "Saving grasp to",graspFile
        f = open(graspFile,'w')
        jsonobj = grasps[i].writeToDict()
        json.dump(jsonobj,f)
        f.close()

else:
    #save grasp to disk
    graspFile = os.path.join(apc.item_model_path,objectname,"grasps",grippername,editedGrasp.name+".json")
    print "Saving grasp to",graspFile
    f = open(graspFile,'w')
    jsonobj = editedGrasp.writeToDict()
    json.dump(jsonobj,f)
    f.close()

