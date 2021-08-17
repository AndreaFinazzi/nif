# Global Parameters Handling

A 'global parameters server' is available in the `nif_common_nodes` package.
NodeParametersNode handles the global parameters available to all nodes (through an interfacing parameter client available in `IBaseNode`), so that the common parameters doesn't have to be passed to each node locally. 

The global parameters configuration file can be pass at launch time as LaunchArgument (see `nif_common_nodes/launch/parameters.launch.py`).

An instance of the server with default parameters can be launched with:

```shell 
ros2 launch nif_common_nodes parameters.launch.py
```

## Global parameters interface
A copy of a global parameter can be retrieved calling  
`T IBaseNode::get_global_parameter<T>(const std::string &)`  
from each node implementation extending `IBaseNode` or one of its children (`IBaseSynchronizedNode`, `IControlNode`, etc.).  

Similarly,  
`void IBaseNode::set_global_parameter(const std::string &, T)`  
is available to set a global parameter through the same client.

Note that the getter and the setter mentioned above throw `rclcpp::exceptions::ParameterNotDeclaredException` if the global parameter is not defined (getter only), and `std::runtime_exception` if the server is unavailable, thus the caller should define a proper strategy to handle miscalls.

## Global parameters naming

Global parameters are defined in a `.yaml` file passed at launch time. The naming in the params file should be coherent with the one defined in  
`nif::common::constants::parameters::names` in `nif_common/constants.h` (and vice-versa).

In the constants file, each parameter name is in the format `<namespace>.<param_name>`.  
For instance, to the following `.yaml` structure:
```yaml 
/**:
  ros__parameters:
    frames:
      body: "rear_axle_middle"
      global: "map"
```

Corresponds the following constants:
```c++
namespace names {

/**
 * Name for the body frame id parameter.
 */
constexpr const char* FRAME_ID_BODY = "frames.body";

/**
 * Name for the global frame id parameter.
 */
constexpr const char* FRAME_ID_GLOBAL = "frames.global";

} // namespace names

```
