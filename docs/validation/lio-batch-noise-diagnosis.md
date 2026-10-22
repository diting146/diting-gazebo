# Instantaneous scan update diagnosis

The Gazebo cloud has constant acquisition time: every point time is 1 microsecond. This produces a full-frame batch in Point-LIO, rather than the small per-point groups used by a physical spinning scanner.

In `point_lio/include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp`, `update_iterated_dyn_share_modified()` adds `M_Noise` as variance when measurement rows are fewer than the state dimension. For a large batch it instead multiplies H-transpose-H and H-transpose by `M_Noise`, i.e. treats it as information (inverse variance). `Estimator.cpp` passes `mapping.lidar_meas_cov` directly into this value.

The old value 0.001 therefore substantially weakened the lidar correction for full-frame updates. Candidate custom configuration: disable the PCL voxel filter to preserve exactly equal timestamps and a single large batch; set `lidar_meas_cov: 1000.0` (inverse of 0.001) for this fork's large-batch branch. Keep time=1 microsecond and timestamp_unit=2; do not invent a scan sweep. Also match acc_norm to the measured simulator gravity 9.8 m/s².

No upstream C++ or launch changes. Runtime spin acceptance is recorded separately; this source diagnosis alone does not constitute a pass.
