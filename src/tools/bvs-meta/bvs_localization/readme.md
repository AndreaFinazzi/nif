# BVS Localization
The purpose of this package is to provide interfaces to input multiple GPS / velocity odometry sources and itelligently select / combine them into a single frame.

# Params:
The are the bare mimimum params of the node. Depending on the config_source more params may also need to be set for operation.
```
~ltp_latitude: origin of the ltp frame
~ltp_longitude: origin of the ltp frame
~ltp_altitude: origin of the ltp frame
~ltp_frame: the name of the ltp frame to maintain with a tf
~vehicle_frame: some static frame on the vehicle to put odometry in
~config_source: `param` (currently must be set to param, see Parameter Loading below)
```

# [Estimate](include/bvs_localization/estimate/estimate.h)
This package is only able to combine sources to estimate position, orientation, and linear velocity. Sources are able to select which they provide within their estimate. The disparate estimates are selected based on health (GOOD, BAD, STALE) and priority (eg: rear_wheels are higher priority than inspva) to be merged into a single estimate. (see [EstimateMerger](include/bvs_localization/estimate/estimate_merger.h)).

The current metric for determining priority to merge is if an estimate is in better health it is selected over those in lower health. If two estimates have the same health then the priority determines which is selected.

NOTE: while many sources can be monitored and published, only sources specified in source_priority will be merged into the central estimate.

# [Sources](include/bvs_localization/estimate/estimate_source.h)
The interface for a source is simple, a source implements the EstimateSource as an interface to provide its estimate / health. The EstimateSource class also contains health / info publishers to run each cycle.

## [Source Factory](include/bvs_localization/sources/source_factory.h)
The source factory abstracts away configuration loading and source generation to a single place, for creation logic look here.

Every source requires the following params:
```
type: <type of the source as detailed below>
```

## Parameter Loading
As noted configuration loading is abstracted to the source factory, it should eventually be able to load params from either ros2 param or a yaml file.
Currently only ros2 param loading is implemented.

To set which source to use set the `~config_source` parameter. If set to `param` it will load from ros2 params.

### ROS2 Params
When loading from ROS2 Params the following parameters must be set in addition to the ones described in the Params section.

```
~sources: [...], an array of sources to load in.
~source_priority: the priority of sources to use when merging into a single estimate (see Estimate section)
```
Any source named in the `sources` parameter will expect more parameters to be set. These will be in the format:
```
~sources.<source_name>.<source_param>
```

Every source will expect the `type` source param to be set - so for a source of name `default_input` the expected rosparam is
```
~sources.default_input.type
```
Options for this parameter are the source types named below. Depending on what this is set to, more parameters as detailed below will be expected.

## Source Types
### [novatel_oem7_inspva](include/bvs_localization/sources/novatel_oem7_inspva.h)
This source inputs the oem7 inspva messages to generate a position, orientation and velocity estimate.
Source Params:
```
topic: <the topic to received inspva messages on, should be of type novatel_oem7_msgs/msg/INSPVA>
health_lim_age: <seconds since the last update to consider this estimate stale>
```

### [rear_wheel_velocity](include/bvs_localization/sources/rear_wheel_velocity.h)
This source inputs wheel velocity reports to generate a linear_x velocity estimate.
Source Params:
```
topic: <the topic to look for wheel velocity estimates, should be of type raptor_dbw_msgs/msg/WheelSpeedReport>
```

### [front_wheel_velocity](include/bvs_localization/sources/rear_wheel_velocity.h)
This source inputs wheel velocity reports to generate a linear_x velocity estimate.
Source Params:
```
topic: <the topic to look for wheel velocity estimates, should be of type raptor_dbw_msgs/msg/WheelSpeedReport>
```

### [novatel_oem7_bestpos](include/bvs_localization/sources/novatel_oem7_bestpos.h)
This source inputs oem7 bestpos messages to generate a position estimate.
Source Params:
```
topic: <the topic to receive bestpos messages on, should be of type novatel_oem7_msgs/msg/BESTPOS>
health_lim_stddev: <limit for the lat and long stddev to consider estimate bad>
health_lim_age: <seconds since the last update to consider this estimate stale>
```

### [novatel_oem7_bestvel](include/bvs_localization/sources/novatel_oem7_bestvel.h)
This source inputs oem7 bestpos messages to generate a position estimate.
Source Params:
```
topic: <the topic to receive bestpos messages on, should be of type novatel_oem7_msgs/msg/BESTPOS>
orient_to_inspva: <boolean, whether or not to use the inspva orientation to project the horizontal speed in the correct direction, if false it will assume the horizontal velocity is forward>
health_lim_age: <seconds since the last update to consider this estimate stale>
```