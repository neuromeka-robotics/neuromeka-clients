# indyeye_client.py for IndyEye 0.5.0
# Custom version-2023/02/23
import sys
import os

import requests
from PIL import Image
import json
import socket
from io import BytesIO
import pickle
import time

import grpc
from .proto.EyeTask_pb2_grpc import EyeTaskStub
from .proto import EyeTask_pb2

ConnectUI = '/main/connect'
GetVersion = '/main/version'
GetApp = '/main/app'
RobotStatus = '/main/robot/status'
DisplayImage = '/main/robot/image'
ImageSize = '/main/robot/image/size'
RobotList = '/main/robot/info'
RobotConnect = '/main/robot/connect'
RobotDisconnect = '/main/robot/disconnect'
CameraIndex = '/main/camera/index'
GetCameraCount = '/main/camera/count'
CameraParameters = '/main/camera/param'
CameraAutoset = '/main/camera/autoset'
LogList = "/main/log/list"
PopMessage = "/message/popup"
Reboot = "/main/reboot"
License = "/main/license"
Locks = "/main/locks"

CamMode = '/calibration/camera/mode'
DetectMarker = '/calibration/camera/marker'
Viewpoint = '/calibration/viewpoint'
PoseNumber = '/calibration/viewpoint/count'
CalibationFileLIst = '/calibration/file/list/{type}'
CalibrationFile = '/calibration/file/{type}'
CalibrationDefault = '/calibration/default/{type}'
CalibrationStart = '/calibration/start'
CalibrationStop = '/calibration/stop'
CalibrationProgress = '/calibration/progress'
ShowWorkspace = '/calibration/camera/workspace'
DetectWorkspace = '/calibration/workspace/detect'
EditWorkspace = '/calibration/workspace/edit/{type}/{axis}'

GraphShow = '/detect/show'
GraphRun = '/detect/run'
GraphFileList = '/detect/file/list/graph'
GraphFile = '/detect/file/graph'
GraphDefault = '/detect/graph/default'
GraphEdit = '/detect/graph/{index}/{type}'
GraphFlow = '/detect/graph/flow'
GraphError = '/detect/graph/error'
GraphDictionary = '/detect/graph/wizard/dictionary/{lang}'
GraphWizard = '/detect/graph/wizard'
GraphTemplate = '/detect/graph/template'

ModuleFileList = '/detect/file/list/module/{index}'
ModuleFile = '/detect/file/module/{index}'
ModuleDictList = '/detect/module/dict/list'
ModuleHelpCheck = '/detect/module/help/check/{name}/{lang}'
ModuleHelpDocs = '/detect/module/help/docs/{name}/{lang}'
ModuleIcon = '/detect/module/icon/{name}'
ModuleAsset = '/detect/module/asset/{name}/{path:path}'
ModuleParam = '/detect/module/params/{index}/{type}'

GripperMark = '/detect/module/gripper/mark/{type}'
GripperDraw = '/detect/module/gripper/draw/{type}'
GripperList = '/detect/module/gripper/list'
GripperPickTest = '/detect/module/gripper/picktest'

CadFileList = '/cad/file/list'
CadFile = '/cad/file'

EYE_PORT_TCP = 2002  # default IndyEye Task Server Port
EYE_PORT_GRPC = 10511
custom_dat_len = 32  # default custom data length


class EyeCommand:
    DETECT = 0  # detect command
    RETRIEVE = 1  # no-detect, retrieve results one-by-one, with Select module ON RETRIEVE Option
    RESET = 2  # not used now
    GETLIST = 3  # get class list


class DetectKey:
    DETECTED = 'detected'
    PASSED = 'passed'
    CLASS = 'cls'
    REF_TO_END_TOOL = 'tar_ee_pose'       # 6D task position to pick object.
    REF_TO_PICK_POINT = 'tar_tool_pose'   # 6D Tool Center Position to pick object.
    REF_TO_OBJECT = 'tar_obj_pose'        # 6D Position of detected object.
    TOOL_INDEX = 'tool_idx'
    ERROR_STATE = 'error_state'
    ERROR_MODULE = 'error_module'
    CLASS_NAMES = 'class_names'

##
# @class IndyEyeClient
# @brief Rest API client
class IndyEyeClient:
    def __init__(self, eye_ip):
        self.session = requests.session()
        self.eye_ip = eye_ip
        self.url = f"http://{eye_ip}:8088"
        self.version = self.get_version()
        if float(".".join(self.version.split(".")[:2])) >= 0.5:
            # initialize gRPC
            self.channel = grpc.insecure_channel('{}:{}'.format(eye_ip, EYE_PORT_GRPC),
                                                 options=[('grpc.max_receive_message_length', 100 * 1024 * 1024)])
            self.stub = EyeTaskStub(self.channel)
            self.request_id = 0
            self.run_command = self.run_command_grpc
        else:
            self.run_command = self.run_command_tcp

    ##
    # @brief indyeye communication function
    # @param cmd        command id, 0~3
    # @param cls        index of target class to detect. 0: ALL, 1~: index of class
    # @param pose_cmd   end-effector pose, [x,y,z,u,v,w] (unit: m, deg)
    # @param robot_ip   ip address of robot, for multi-robot use
    def run_command_grpc(self, cmd, cls=0, pose_cmd=None, robot_ip=None, **kwargs):
        self.request_id += 1
        if cmd == EyeCommand.GETLIST:
            resp = self.stub.GetClassList(EyeTask_pb2.Request(id=self.request_id))
            return {DetectKey.CLASS_NAMES: resp.class_names,
                    DetectKey.ERROR_STATE: resp.error_state}
        else:
            sdict = {'id': self.request_id, 'cls': int(cls)}
            if cmd == EyeCommand.DETECT:
                if pose_cmd is not None:
                    sdict['pose_cmd'] = pose_cmd
                if robot_ip is not None:
                    sdict['robot_ip'] = robot_ip
                resp = self.stub.Detect(EyeTask_pb2.DetectRequest(**sdict))
            elif cmd == EyeCommand.RETRIEVE:
                if robot_ip is not None:
                    sdict['robot_ip'] = robot_ip
                resp = self.stub.Retrieve(EyeTask_pb2.RetrieveRequest(**sdict))
            else:
                raise(NotImplementedError("Unknown command {}".format(cmd)))
            return {DetectKey.DETECTED: resp.detected,
                    DetectKey.PASSED: resp.passed,
                    DetectKey.CLASS: resp.cls,
                    DetectKey.REF_TO_END_TOOL: resp.tar_ee_pose,
                    DetectKey.REF_TO_PICK_POINT: resp.tar_tool_pose,
                    DetectKey.REF_TO_OBJECT: resp.tar_obj_pose,
                    DetectKey.TOOL_INDEX: resp.tool_idx,
                    DetectKey.ERROR_STATE: resp.error_state,
                    DetectKey.ERROR_MODULE: resp.error_module}

    ##
    # @brief indyeye communication function
    # @param cmd        command id, 0~3
    # @param cls        index of target class to detect. 0: ALL, 1~: index of class
    # @param pose_cmd   end-effector pose, [x,y,z,u,v,w] (unit: m, deg)
    # @param robot_ip   ip address of robot, for multi-robot use
    def run_command_tcp(self, cmd, cls, pose_cmd=None, robot_ip=None, **kwargs):
        sock = socket.socket(socket.AF_INET,
                             socket.SOCK_STREAM)  # SOCK_STREAM is TCP socket

        try:
            sock.connect((self.eye_ip, EYE_PORT_TCP))
            sdict = {'command': int(cmd), 'class_tar': int(cls), }
            if pose_cmd is not None:
                sdict['pose_cmd'] = pose_cmd
            if robot_ip is not None:
                sdict['robot_ip'] = robot_ip
            sdict.update(kwargs)
            sjson = json.dumps(sdict, cls=NumpyEncoder)
            sbuff = sjson.encode()
            sock.send(sbuff)
            #             print('sent: ',sjson)

            rbuff = sock.recv(1024)
            rjson = "".join(map(chr, rbuff))
            rdict = json.loads(rjson)
            if cmd == EyeCommand.GETLIST:
                rdict = {DetectKey.CLASS_NAMES: rdict['class_list'],
                         DetectKey.ERROR_STATE: rdict['STATE']}
            elif cmd in [EyeCommand.DETECT, EyeCommand.RETRIEVE]:
                rdict = {DetectKey.DETECTED: rdict['class_detect'] > 0,
                         DetectKey.PASSED: rdict['class_detect'] > 0 and rdict['STATE'] == 0,
                         DetectKey.CLASS: rdict['class_detect'],
                         DetectKey.REF_TO_END_TOOL: rdict['Tbe'],
                         DetectKey.REF_TO_PICK_POINT: rdict['Tbt'],
                         DetectKey.REF_TO_OBJECT: rdict['Tbo'],
                         DetectKey.TOOL_INDEX: rdict['tool_idx'],
                         DetectKey.ERROR_STATE: rdict['STATE']!=0,
                         DetectKey.ERROR_MODULE: None}
            else:
                raise(NotImplementedError("Unknown command {}".format(cmd)))
            return rdict
        except Exception as e:
            print("[ERROR] IndyEye Communication:")
            print(e)
        finally:
            sock.close()

    # '''
    # @param cls : object class (use get_object_list to get cls). 0: ALL, 1~: index of class
    # @param task_pos : current task position, [x,y,z,u,v,w] (unit: m, deg)
    # @return {
    #     #              'Tbe' : pick pose of end-effector in base coordinates
    #     #              'Tbt' : pick pose of tool point in base coordinates
    #     #              'Tbo' : detected object pose in base coordinates
    #     #             }
    def detect(self, cls, task_pos, mode=DetectKey.REF_TO_END_TOOL, **kwargs):
        rdict = self.run_command(cmd=EyeCommand.DETECT, cls=cls, pose_cmd=task_pos, **kwargs)
        if rdict is not None and rdict[DetectKey.DETECTED]:
            return rdict[mode]
        else:
            return None

    def detect_by_object_name(self, target_name, task_pos, mode=DetectKey.REF_TO_END_TOOL, **kwargs):
        '''
            target_name : object name (requires name of object from get_object_list),
            task_pos : current task position
        '''
        objs = self.get_object_dict()
        found_target = None
        found_target_idx = -1
        for key in objs.keys():
            if target_name == objs[key]:
                found_target_idx = key
        if found_target_idx != -1:
            result = self.run_command(cmd=EyeCommand.DETECT, cls=found_target_idx, pose_cmd=task_pos, **kwargs)
            if rdict is not None and rdict[DetectKey.DETECTED]:
                found_target = result[mode]
            else:
                found_target = None
        return found_target

    def retrieve(self, cls, task_pos=None, mode=DetectKey.REF_TO_END_TOOL, **kwargs):
        '''
            cls : object class ( use get_object_list to get cls),
            cls = 0 : all,

            task_pos : current task position
        '''
        rdict = self.run_command(cmd=EyeCommand.RETRIEVE, cls=cls, pose_cmd=task_pos, **kwargs)
        if rdict is not None and rdict[DetectKey.DETECTED]:
            return rdict[mode]
        else:
            return None

    def get_object_dict(self, **kwargs):
        rdict = self.run_command(cmd=EyeCommand.GETLIST, cls=0, pose_cmd=None, **kwargs)
        if rdict is not None and DetectKey.CLASS_NAMES in rdict:
            return {i_o+1: obj for i_o, obj in enumerate(rdict[DetectKey.CLASS_NAMES])}
        else:
            return None

    def get_version(self):
        return self.get(GetVersion)['data']

    ##
    # @return RGB image
    def image(self):
        resp = self.session.get(self.url + DisplayImage)
        binarycontent = BytesIO(resp.content)
        img = Image.open(binarycontent)
        return img

    ##
    # @return RGB image in local repository
    def repo_image(self, filename):
        resp = self.session.get(self.url + f'/download/image/{filename}.png')
        binarycontent = BytesIO(resp.content)
        img = Image.open(binarycontent)
        return img

    def get(self, command, timeout=3, **kwargs):
        resp = self.session.get(self.url + command, params=kwargs, timeout=timeout)
        return resp.json()

    def get_pkl(self, command, timeout=30, **kwargs):
        resp = self.session.get(self.url + command, params=kwargs, timeout=timeout)
        return pickle.load(pickle.io.BytesIO(resp.content))

    def put_pkl(self, command, timeout=30, **kwargs):
        resp = self.session.put(self.url + command, params=kwargs, timeout=timeout)
        return pickle.load(pickle.io.BytesIO(resp.content))

    def post(self, command, timeout=3, **kwargs):
        resp = self.session.post(self.url + command, params=kwargs, timeout=timeout)
        return resp.json()

    def put(self, command, timeout=3, **kwargs):
        resp = self.session.put(self.url + command, params=kwargs, timeout=timeout)
        return resp.json()

    def delete(self, command, timeout=3, **kwargs):
        resp = self.session.delete(self.url + command, params=kwargs, timeout=timeout)
        return resp.json()

    def add_module(self, index, type, name, verbose=True):
        check_error(self.post(GraphEdit.format(index=index, type=type), name=name),
                    f"Add Module {name}", verbose=verbose)
        graphflow = self.get(GraphFlow)['data']

        if str(index) not in graphflow["graph"]:
            return graphflow["starting_node"]
        else:
            ref_node = graphflow['graph'][str(index)]
            if type == "next":
                return ref_node["out"]
            elif type == "branch":
                return ref_node["branch"]
            elif type == "condition":
                return ref_node["condition"]
            else:
                raise (NotImplementedError(f"unknown connection type {type}"))

    def get_module_params(self, index, timeout=3):
        return self.get(ModuleParam.format(index=index, type='value'), timeout=timeout)['data']

    def get_module_param_range(self, index, param, timeout=1):
        return self.get(ModuleParam.format(index=index, type='available'), param=param, timeout=timeout)['data'][
            'available']

    def set_module_params(self, index, timeout=3, verbose=False, **kwargs):
        for param, value in kwargs.items():
            check_error(
                self.post(ModuleParam.format(index=index, type='value'), param=param, value=value, timeout=timeout),
                f"Param Set {param}={value}", verbose=verbose)

    def del_module(self, index, verbose=True):
        check_error(self.delete(GraphEdit.format(index=index, type="None")), "Delete Module", verbose=verbose)

    def clear_graph(self, verbose=True):
        check_error(self.delete(GraphEdit.format(index="all", type="None")), "Clear Graph", verbose=verbose)



##
# @brief check communication error and print [OK] message or raise error
def check_error(resp, message, verbose=True):
    if resp['error_code'] == 0:
        if verbose:
            print("[OK] " + message)
        return resp
    else:
        raise (RuntimeError(f"[ERROR {resp['error_code']}] " + message))

