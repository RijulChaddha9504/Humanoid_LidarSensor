from omni.isaac.kit import SimulationApp

# Launch the simulator
simulation_app = SimulationApp({"headless": False})

# Now import Isaac Lab components
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicCuboid, DynamicCylinder
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf

# Initialize the world
world = World(stage_units_in_meters=1.0)
stage = world.stage

# Define asset paths
assets_root_path = get_assets_root_path()
robot_usd_path = f"{assets_root_path}/Isaac/Robots/Jackal/jackal.usd"  # Change this if you want another robot
lidar_config = "Example_Rotary"  # Lidar preset config name

# Load the robot into the world
robot_prim_path = "/World/Robot"
add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

# Attach a lidar sensor
import omni.kit.commands
_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=f"{robot_prim_path}/Lidar",
    parent=robot_prim_path,
    config=lidar_config,
    translation=Gf.Vec3f(0.0, 0.0, 0.3),
    orientation=Gf.Vec3f(0.0, 0.0, 0.0),
)

# Add obstacles
obstacle1 = DynamicCuboid(
    prim_path="/World/Obstacle1",
    position=(2.0, 0.0, 0.5),
    size=(1.0, 1.0, 1.0),
    color=(1.0, 0.2, 0.2)
)

obstacle2 = DynamicCylinder(
    prim_path="/World/Obstacle2",
    position=(-1.5, 1.5, 0.5),
    radius=0.5,
    height=1.0,
    color=(0.2, 0.3, 1.0)
)

# Initialize simulation
world.reset()

# Run the simulation loop
while simulation_app.is_running():
    world.step(render=True)

# Cleanup
simulation_app.close()
