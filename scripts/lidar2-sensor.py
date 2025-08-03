from omni.isaac.kit import SimulationApp
# Headless or GUI mode
simulation_app = SimulationApp({"headless": False})
# --- your existing code here ---
import omni.kit.commands
from pxr import Gf
import omni.replicator.core as rep

lidar_config = "Example_Rotary"

_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensor",
    parent=None,
    config=lidar_config,
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(1,0,0,0),
)
render_product = rep.create.render_product(sensor.GetPath(), [1, 1])
annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
annotator.attach(render_product)

writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer.attach(render_product)

