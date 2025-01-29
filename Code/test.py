from pxr import Usd, UsdGeom
import omni.usd
import omni.graph as og

#stage = omni.usd.get_context().get_stage()


ogn = og.core.get_node_by_path("/World/Robot/Tower/Axis2/gripper/SurfaceGripperActionGraph/open")

attr = ogn.get_attribute("state:enableImpulse")
attr.set(1)
ogn.request_compute()