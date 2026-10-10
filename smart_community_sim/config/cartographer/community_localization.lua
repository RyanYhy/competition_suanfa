include "community_2d.lua"
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
return options
