#!/bin/bash
python3 generate_sphere_urdf.py \
    ../assets/gp4_lego/gp4.urdf \
    --output ../assets/gp4_lego/gp4_spherized.urdf \
    --base_link 8 \
    --link_1 8 \
    --link_3 8 \
    --link_5 8 \
    --fts 8 \
    --depth 2 \
    --branch 8 \
    --link_2_depth 2 \
    --link_4_depth 2 \
    --link_tool 8 \
    --link_tool_depth 3 \
    --volume_heuristic_ratio 0.7

# convert urdf to spheres for visualization
python3 visual_from_collision.py ../assets/gp4_lego/gp4_spherized.urdf

# visualize
python3 visualize_urdf.py ../assets/gp4_lego/gp4_spherized_visual_from_collision.urdf
