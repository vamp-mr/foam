import argparse
import time
from itertools import cycle

import pybullet as p
import pybullet_data


_DEFAULT_COLORS = [
    [0.94, 0.33, 0.33, 0.35],
    [0.28, 0.58, 0.89, 0.35],
    [0.38, 0.76, 0.35, 0.35],
    [0.93, 0.73, 0.30, 0.35],
    [0.69, 0.45, 0.90, 0.35],
    [0.27, 0.78, 0.78, 0.35],
]


def _collect_sphere_collisions(body_id, sphere_scale):
    """Create visual-only spheres for any spherical collision geometry."""
    spheres = []
    link_indices = [-1] + list(range(p.getNumJoints(body_id)))
    color_cycle = cycle(_DEFAULT_COLORS)
    for link_index in link_indices:
        link_color = next(color_cycle)
        for shape in p.getCollisionShapeData(body_id, link_index):
            geom_type = shape[2]
            if geom_type != p.GEOM_SPHERE:
                continue

            radius = shape[3][0]
            local_pos = shape[5]
            local_orn = shape[6]

            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=radius * sphere_scale,
                rgbaColor=link_color,
            )
            sphere_body = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_shape,
                baseCollisionShapeIndex=-1,
                basePosition=[0, 0, 0],
                useMaximalCoordinates=True,
            )
            spheres.append(
                {
                    "body": sphere_body,
                    "link_index": link_index,
                    "local_pos": local_pos,
                    "local_orn": local_orn,
                    "color": link_color,
                    "radius": radius * sphere_scale,
                }
            )

    return spheres


def _update_sphere_transforms(robot_id, spheres):
    """Keep the visual-only spheres aligned with their parent link."""
    for sphere in spheres:
        link_index = sphere["link_index"]
        if link_index == -1:
            link_pos, link_orn = p.getBasePositionAndOrientation(robot_id)
        else:
            link_state = p.getLinkState(robot_id, link_index, computeForwardKinematics=True)
            link_pos, link_orn = link_state[4], link_state[5]

        world_pos, world_orn = p.multiplyTransforms(
            link_pos,
            link_orn,
            sphere["local_pos"],
            sphere["local_orn"],
        )
        p.resetBasePositionAndOrientation(sphere["body"], world_pos, world_orn)


def _hide_visual_geometry(body_id):
    for visual_data in p.getVisualShapeData(body_id):
        body_unique_id = visual_data[0]
        link_index = visual_data[1]
        p.changeVisualShape(body_unique_id, link_index, rgbaColor=[0, 0, 0, 0])


def load_urdf(
    urdf_path,
    use_gui=True,
    show_spheres=False,
    sphere_scale=1.0,
    hide_visuals=False,
):
    # Start PyBullet simulation
    if use_gui:
        physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set search path
    p.setGravity(0, 0, -9.81)  # Set gravity

    # Load plane and robot URDF
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

    if hide_visuals:
        _hide_visual_geometry(robot_id)

    spheres = _collect_sphere_collisions(robot_id, sphere_scale) if show_spheres else []
    if spheres:
        _update_sphere_transforms(robot_id, spheres)
    
    # Run the simulation loop
    try:
        while use_gui:
            p.stepSimulation()
            if spheres:
                _update_sphere_transforms(robot_id, spheres)
            
            # Get the current position of the robot's base
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            # If the robot's base goes below the plane (z < 0), reset its z-position to 0.
            if pos[2] < 3:
                p.resetBasePositionAndOrientation(robot_id, [pos[0], pos[1], 0], orn)
            
            time.sleep(1 / 240.0)  # Step simulation at ~240Hz
    except KeyboardInterrupt:
        pass
    
    # Disconnect from PyBullet
    p.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load a URDF file into PyBullet simulation.")
    parser.add_argument("urdf_file", type=str, help="Path to the URDF file.")
    parser.add_argument("--nogui", action="store_true", help="Run without GUI.")
    parser.add_argument(
        "--show-spheres",
        action="store_true",
        help="Overlay spherical collision geometry on top of mesh visuals.",
    )
    parser.add_argument(
        "--sphere-scale",
        type=float,
        default=1.0,
        help="Multiply collision sphere radii by this factor for visualization.",
    )
    parser.add_argument(
        "--hide-visuals",
        action="store_true",
        help="Hide the mesh visuals to inspect only the sphere overlay.",
    )
    args = parser.parse_args()

    load_urdf(
        args.urdf_file,
        use_gui=not args.nogui,
        show_spheres=args.show_spheres,
        sphere_scale=args.sphere_scale,
        hide_visuals=args.hide_visuals,
    )
