[![DevEnvironment](https://github.com/AndreaFinazzi/nif/actions/workflows/colcon-test.yml/badge.svg)](https://github.com/AndreaFinazzi/nif/actions/workflows/colcon-test.yml)

# KAIST team software stack for the Indy Autonomous Challenge.


## Tracing notes
**Autoware_perf**: 
- Paper:    https://www.sciencedirect.com/science/article/pii/S1383762121002344?ref=pdf_download&fr=RR-2&rr=7117037f9c7412e2
- Code:     https://github.com/azu-lab/ROS2-E2E-Evaluation

A tracing docker image is defined in `./docker/tracing`. Usage:
- Build image `./nif_image_build --tracing --push`
- Start ade container `./nifstart [--cuda-host] --tracing`
- Enter the ade environment `ade enter`
- In parallel
  - Terminal 1: Start the tracing session `./tracing/niftracing` (Press Enter) -> note down the *tracelog path(1)*.
  - Terminal 2: Run a ROS2 stack e.g. `TRACK=LVMS ros2 launch nif_launch deploy.launch.py`
  - Run simulator/bag commands, depending on the experiment (to feed the stack with realistic data).

Trace analisys:
- Parse the tracelog `./tracing/niftracing-analisys <tracelog path(1)> <output path(2)>`
- Open and run the `e2elatency.jpynb` or a custom script to process the data.

### e2elatency.jpynb
This notebook generates end-to-end latency of a desired execution path (node->topic->node->topic->...).
The `path_prefix` variable should be set to the `<output path(2)>` path (See previous section).

The execution path is defined by the node/topic pairs passed to the `cbname(..)` function. For instance:
```py
cb1 = cbname(node_awl,topic_sen_gnss_top)
cb2 = cbname(node_ssm,topic_ns_awl)
cb3 = cbname(node_csl,topic_ss)
cb4 = cbname(node_ssc,topic_joy_brk)
```
Defines the path `node_awl->node_ssm->node_csl->node_ssc` through the specified topics.

### Troubleshooting:
to debug a jpynb in VSCode:
```bash
pip install --upgrade ipykernel ipython
```