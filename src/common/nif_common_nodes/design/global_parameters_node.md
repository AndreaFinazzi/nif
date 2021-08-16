NodeParametersNode handles the global parameters available to all nodes (through `IBaseNode`), so that the common parameters doesn't have to be passed to each node locally.

The global parameters configuration file can be pass at launch time as LaunchArgument (see `nif_common_nodes/launch/parameters.launch.py`).

A single global parameter can be retrieved calling `IBaseNode::get_global_parameter(const std::string & param_name)` from each node implementation.