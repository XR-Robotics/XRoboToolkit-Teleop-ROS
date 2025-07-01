"""
Uses Mujoco to convert from URDF to MJCF files.

Modified from urdf2mjcf https://github.com/kscalelabs/urdf2mjcf.

"""

import argparse
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Union

import mujoco

from urdf2mjcf.utils import iter_meshes, save_xml
from urdf2mjcf.convert import (add_compiler, add_default, add_option,
        add_assets, add_root_body, add_worldbody_elements, add_actuators,
        add_sensors, add_cameras, add_visual_geom_logic)


def convert_urdf_to_mjcf(
    urdf_path: Union[str, Path],
    mjcf_path: Union[str, Path, None] = None,
    no_collision_mesh: bool = False,
    copy_meshes: bool = False,
    camera_distance: float = 3.0,
    camera_height_offset: float = 0.5,
    no_frc_limit: bool = False,
    mesh_dir: str = None,
) -> None:
    """Convert a URDF file to an MJCF file.

    Args:
        urdf_path: The path to the URDF file.
        mjcf_path: The path to the MJCF file. If not provided, use the URDF
            path with the extension replaced with ".xml".
        no_collision_mesh: Do not include collision meshes.
        copy_meshes: Copy mesh files to the output MJCF directory if different
            from URDF directory.
        camera_distance: Distance of the fixed camera from the robot.
        camera_height_offset: Height offset of the fixed camera from the robot.
        no_frc_limit: Do not include force limit for the actuators.
    """
    urdf_path = Path(urdf_path)
    mjcf_path = Path(mjcf_path) if mjcf_path is not None else urdf_path.with_suffix(".xml")
    if not Path(urdf_path).exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    mjcf_path.parent.mkdir(parents=True, exist_ok=True)
    if mesh_dir:
        mesh_dir = Path(mesh_dir)

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)

        # Copy URDF file to temp directory
        urdf_dir = urdf_path.parent.resolve()
        temp_urdf_path = temp_dir_path / urdf_path.name
        temp_urdf_path.write_bytes(urdf_path.read_bytes())

        # Copy mesh files to temp directory and potentially to output directory
        mesh_files = []
        for (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path):
            for mesh_path in list({visual_mesh_path, collision_mesh_path}):
                if mesh_path is not None:
                    temp_mesh_path = temp_dir_path / mesh_path.name
                    if not mesh_path.exists() and mesh_dir:
                        mesh_path = mesh_dir / mesh_path.name
                    try:
                        temp_mesh_path.symlink_to(mesh_path.resolve())
                        if mesh_dir:
                            mesh_files.append(mesh_path)
                        else:
                            mesh_files.append(mesh_path.relative_to(urdf_dir))
                    except FileExistsError:
                        pass

        urdf_tree = ET.parse(temp_urdf_path)
        for mesh in urdf_tree.iter("mesh"):
            full_filename = mesh.attrib.get("filename")
            if full_filename is not None:
                mesh.attrib["filename"] = Path(full_filename).name

        # Load the URDF file with Mujoco and save it as an MJCF file in the temp directory
        temp_mjcf_path = temp_dir_path / mjcf_path.name

        # Get the distance to the lowest point on the robot, to offset the root body.
        model = mujoco.MjModel.from_xml_path(temp_urdf_path.as_posix())
        data = mujoco.MjData(model)
        mujoco.mj_fwdPosition(model, data)
        foot_distance = -(data.geom_xpos[:, 2] - model.geom_size[:, 2]).min()

        mujoco.mj_saveLastXML(temp_mjcf_path.as_posix(), model)

        # Read the MJCF file and update the paths to the meshes
        mjcf_tree = ET.parse(temp_mjcf_path)
        root = mjcf_tree.getroot()

        for asset in mjcf_tree.iter("asset"):
            for mesh in asset.iter("mesh"):
                mesh_name = Path(mesh.attrib["file"]).name
                # Update the file attribute to just the mesh name
                mesh.attrib["file"] = mesh_name

        if no_frc_limit:
            for joint in root.iter("joint"):
                if "actuatorfrcrange" in joint.attrib:
                    del joint.attrib["actuatorfrcrange"]

        # Turn off internal collisions
        if not no_collision_mesh:
            for geom in root.iter("geom"):
                geom.attrib["contype"] = str(1)
                geom.attrib["conaffinity"] = str(0)
                geom.attrib["density"] = str(0)
                geom.attrib["group"] = str(1)

        # Manually set additional options.
        add_default(root)
        add_compiler(root)
        add_option(root)
        add_assets(root)
        add_cameras(root, foot_distance, distance=camera_distance, height_offset=camera_height_offset)
        add_root_body(root, foot_distance)
        add_worldbody_elements(root)
        add_actuators(root, no_frc_limit)
        add_sensors(root)
        add_visual_geom_logic(root)

        # Copy mesh files to the output directory.
        if copy_meshes:
            for mesh_file in mesh_files:
                if mesh_dir:
                    mjcf_mesh_path = mjcf_path.parent.resolve() / "meshes" / mesh_file.name
                    mjcf_mesh_path.parent.mkdir(parents=True, exist_ok=True)
                    urdf_mesh_path = mesh_dir / mesh_file.name
                else:
                    mjcf_mesh_path = mjcf_path.parent.resolve() / mesh_file
                    mjcf_mesh_path.parent.mkdir(parents=True, exist_ok=True)
                    urdf_mesh_path = urdf_dir / mesh_file
                if mjcf_mesh_path != urdf_mesh_path:
                    shutil.copy2(urdf_mesh_path, mjcf_mesh_path)

        # Write the updated MJCF file to the original destination
        save_xml(mjcf_path, mjcf_tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a URDF file to an MJCF file.")
    parser.add_argument("urdf_path", type=str, help="The path to the URDF file.")
    parser.add_argument("--no-collision-mesh", action="store_true", help="Do not include collision meshes.")
    parser.add_argument("--output", type=str, help="The path to the output MJCF file.")
    parser.add_argument("--copy-meshes", action="store_true", help="Copy mesh files to the output MJCF directory.")
    parser.add_argument("--camera-distance", type=float, default=3.0, help="Camera distance from the robot.")
    parser.add_argument("--camera-height-offset", type=float, default=0.5, help="Camera height offset.")
    parser.add_argument("--no-frc-limit", action="store_true", help="Do not include force limit for the actuators.")
    parser.add_argument('--mesh-dir', default=None, type=str, help='mesh(stl) file directory')
    args = parser.parse_args()

    convert_urdf_to_mjcf(
        urdf_path=args.urdf_path,
        mjcf_path=args.output,
        no_collision_mesh=args.no_collision_mesh,
        copy_meshes=args.copy_meshes,
        camera_distance=args.camera_distance,
        camera_height_offset=args.camera_height_offset,
        no_frc_limit=args.no_frc_limit,
        mesh_dir=args.mesh_dir,
    )


if __name__ == "__main__":
    main()
