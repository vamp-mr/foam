#!/usr/bin/env python3

"""Clone collision geometry into the visual section of a URDF."""

from __future__ import annotations

import argparse
import copy
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET


def _replace_visuals_with_collisions(root: ET.Element, only_spheres: bool) -> None:
    name_counters: defaultdict[str, int] = defaultdict(int)

    for link in root.findall("link"):
        collisions = link.findall("collision")
        if not collisions:
            continue

        # drop existing visual geometry
        for visual in list(link.findall("visual")):
            link.remove(visual)

        for idx, collision in enumerate(collisions):
            geometry = collision.find("geometry")
            if geometry is None:
                continue

            has_sphere = geometry.find("sphere") is not None
            if only_spheres and not has_sphere:
                continue

            visual = copy.deepcopy(collision)
            visual.tag = "visual"

            # ensure unique names, especially when multiple spheres approximate a mesh
            link_name = link.get("name", "link")
            if "name" in visual.attrib:
                visual.attrib["name"] = f"{visual.attrib['name']}_visual"
            else:
                name_counters[link_name] += 1
                visual.attrib["name"] = f"{link_name}_visual_{name_counters[link_name]}"

            link.append(visual)


def convert_urdf(
    input_path: Path,
    output_path: Path | None = None,
    only_spheres: bool = False,
) -> Path:
    tree = ET.parse(input_path)
    root = tree.getroot()

    _replace_visuals_with_collisions(root, only_spheres=only_spheres)

    if output_path is None:
        output_path = input_path.with_name(f"{input_path.stem}_visual_from_collision.urdf")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Path to the source URDF.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional output path. Defaults to `<stem>_visual_from_collision.urdf`.",
    )
    parser.add_argument(
        "--only-spheres",
        action="store_true",
        help="Copy only collision entries whose geometry is spherical.",
    )
    args = parser.parse_args()

    output_path = convert_urdf(
        args.input,
        args.output,
        only_spheres=args.only_spheres,
    )
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
