# Marker visualization node
Sends marker data to rviz for graphical rendering.

## Parameters
The following parameters can be used to configure the visualization:
* `default_marker_color_r` (0.0): marker color red value on a scale of 0 to 1
* `default_marker_color_g` (1.0): marker color green value on a scale of 0 to 1
* `default_marker_color_b` (0.0): marker color blue value on a scale of 0 to 1
* `default_marker_color_b` (1.0): marker alpha value on a scale of 0 to 1
* `marker_scale_x` (0.014): x scale of the marker (in meters)
* `marker_scale_y` (0.014): y scale of the marker (in meters)
* `marker_scale_z` (0.014): z scale of the marker (in meters)
* `marker_lifetime` (0.1): duration for which the marker will be visible after receiving marker data (in seconds)
* `marker_frame` (mocap): the origin to which the markers will be rendered relative to, should correspond with the origin used by the mocap system
* `namespace` (mocap\_markers): namespace attached to the visualized markers
* `use_markers_with_id` (true): whether to use Marker messages (if false) or MarkerWithId messages (if true). Needs to correspond with the configuration of the motion capture system driver. Currently markers with ID are only supported by the Qualisys driver.

## Services
The `set_marker_color` service changes the marker color based on the marker ID and specified color. Can be used for highlighting markers when using external tools.
The `reset_marker_color` service resets the color of the marker specified by the marker ID to the default value.
Both services only work when using MarkerWithId messages.
