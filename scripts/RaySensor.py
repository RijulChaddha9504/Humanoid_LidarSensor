import argparse
import torch
import numpy as np
from isaaclab.app import AppLauncher
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
import omni.usd
from isaaclab.sensors import RayCasterCfg, patterns
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab_assets.robots.anymal import ANYMAL_C_CFG

@configclass
class RaycasterSensorSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd",
            scale=(1, 1, 1),
        ),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    ray_caster = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/lidar_cage",
        update_period=1/60,
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.5)),
        mesh_prim_paths=["/World/Ground"],
        ray_alignment="yaw",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=32,
            vertical_fov_range=[-30, 30],
            horizontal_fov_range=[-90, 90],
            horizontal_res=1.0
        ),
        debug_vis=True,
    )

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    while simulation_app.is_running():
        sim.step()
        scene.write_data_to_sim()
        scene.update(sim_dt)
        print(scene["ray_caster"])
        hits = scene["ray_caster"].data.ray_hits_w
        print("Ray hit positions shape:", hits.shape)
        # Optionally save or post-process:
        # arr = hits.cpu().numpy()
        # np.save("ray_hits.npy", arr)

def main():
    parser = argparse.ArgumentParser(description="Example using Isaac Lab RayCaster sensor")
    parser.add_argument("--num_envs", type=int, default=1)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    global simulation_app
    simulation_app = app_launcher.app

    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[3, 3, 3], target=[0.0, 0.0, 0.0])

    scene_cfg = RaycasterSensorSceneCfg(num_envs=args.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] Environment setup complete.")
    run_simulator(sim, scene)
    simulation_app.close()

if __name__ == "__main__":
    main()
