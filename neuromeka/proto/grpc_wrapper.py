import sys
import os

generated_files_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(generated_files_path)

import EtherCATCommgRPCServer_pb2 as EtherCATCommgRPCServer__pb2
from EtherCATCommgRPCServer_pb2_grpc import *

import IndygRPCTask_pb2 as IndygRPCTask__pb2
from IndygRPCTask_pb2_grpc import *

import MobygRPCServer_pb2 as MobygRPCServer__pb2
from MobygRPCServer_pb2_grpc import *

import MotorControlgRPCServer_pb2 as MotorControlgRPCServer__pb2
from MotorControlgRPCServer_pb2_grpc import *