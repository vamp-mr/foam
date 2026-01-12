#!/bin/bash
python3 generate_sphere_urdf.py \
    ../assets/kinova_gen3/kinova_gen3.urdf \
    --output ../assets/kinova_gen3/kinova_gen3_spherized.urdf \
    --base_link 2 \
    --shoulder_link 4 \
    --shoulder_link_depth 1 \
    --half_arm_1_link 4 \
    --half_arm_1_link_depth 1 \
    --half_arm_2_link 4 \
    --half_arm_2_link_depth 1 \
    --forearm_link 3 \
    --forearm_link_depth 2 \
    --spherical_wrist_1_link 3 \
    --spherical_wrist_1_link_depth 2 \
    --spherical_wrist_2_link 3 \
    --spherical_wrist_2_link_depth 2 \
    --bracelet_link 1 \
    --depth 1 \
    --branch 8 \
    --volume_heuristic_ratio 0.7

# convert urdf to spheres for visualization
python3 visual_from_collision.py ../assets/kinova_gen3/kinova_gen3_spherized.urdf

# visualize
python3 visualize_urdf.py ../assets/kinova_gen3/kinova_gen3_spherized_visual_from_collision.urdf
