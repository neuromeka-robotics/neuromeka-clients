from neuromeka import IndyEyeClient
from neuromeka import IndyClient
from neuromeka.eye import *

eye = IndyEyeClient("192.168.0.56")
indy = IndyClient("192.168.0.112")

task_pos = indy.get_task_pos()
object_dict = eye.get_object_dict()
pose_obj = eye.detect(0, task_pos, mode=DetectKey.REF_TO_OBJECT)
pose_ee = eye.detect(0, task_pos, mode=DetectKey.REF_TO_END_TOOL)
