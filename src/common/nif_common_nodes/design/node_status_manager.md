# System Status Manager

SSM package is intended to be a supervisor of the overall health of the system. 
The most intuitive and easy to implement solution is an health check at the node-level, which looks into 

## System-Watchdog decoupling and Single Points of Failure

Of particular importance is the decoupling of the SSM from the rest of the system. 
In fact, it should be avoided by any mean any sort of causality, dependency relation, etc. between the SSM and any other node. 
In other words, the case where a failure in one of the nodes, a failure in a top level process, a failure in the communication between a Node and SSM, would make the SSM fail itself, would make the SSM conceptually useless.

To achieve this, 