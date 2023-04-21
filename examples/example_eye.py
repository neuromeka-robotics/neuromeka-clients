from neuromeka import IndyEyeClient
from neuromeka import IndyClient
from neuromeka.eye import *
from neuromeka.indy_dcp.indydcp_client import *

eye = IndyEyeClient("192.168.0.100")
indy = IndyDCPClient("192.168.0.100", "NRMK-Indy7")

indy.connect()
task_pos = indy.get_task_pos()
object_dict = eye.get_object_dict()
pose_obj = eye.detect(0, task_pos, mode=DetectKey.REF_TO_OBJECT)
pose_ee = eye.detect(0, task_pos, mode=DetectKey.REF_TO_END_TOOL)
indy.disconnect()