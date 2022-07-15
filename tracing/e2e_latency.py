# %%
import numpy as np
import matplotlib.pyplot as plt
import collections
import pandas as pd
import os

# %%
HOME = os.environ.get("HOME")
path_prefix = HOME + "/tracing/tracelog"
session_name = "session-20220704201438"
path_tracelog = os.path.join(path_prefix, session_name + '.txt')
path_start = os.path.join(path_prefix, session_name + '_cstart.txt')
path_end = os.path.join(path_prefix, session_name + '_cend.txt')

path = path_tracelog
f = open(path)
f.close()
with open(path) as f:
    l = f.readlines()

HEADER_PUBLISHERS_E  = 'Publishers:\n' if 'Publishers:\n' in l else 'Publishers (rmw):\n'
HEADER_PUBLISHERS_S  = 'Publishers:\n' if 'Publishers:\n' in l else 'Publishers (rcl):\n'
HEADER_SUBSCRIBERS_E = 'Subscriptions:\n' if 'Subscriptions:\n' in l else 'Subscriptions (rmw):\n'
HEADER_SUBSCRIBERS_S = 'Subscriptions:\n' if 'Subscriptions:\n' in l else 'Subscriptions (rcl):\n'
S = l.index('Nodes:\n')
E_rmw = l.index(HEADER_PUBLISHERS_E)
E = l.index(HEADER_PUBLISHERS_S)
no=[]
for i in l[S+3:E_rmw-1]:
    no.append(i.split()[4])

# %%
import os
dirname = os.path.join(path_prefix, session_name, "out")
os.makedirs(dirname, exist_ok=True)

#plot a label as ms
def label(latency):
    return 'max:{:.02f}ms, mean:{:.02f}ms'.format(max(latency),np.mean(latency))
#plot a graph as ms
def graph(latency, title):
    fig, ax = plt.subplots(1, 1)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    plt.hist(latency, bins=200, label=label(latency))
    plt.legend()
    plt.title("%s"%title, fontsize=15)
    plt.xlabel("Latency [ms]", fontsize=15)
    plt.ylabel("Sample", fontsize=15)
    filename = dirname + "%s"%title+".pdf"
    plt.savefig(filename)

#plot a label as us
def label2(latency):
    return 'max:{:.02f}us, mean:{:.02f}us'.format(max(latency),np.mean(latency))
#plot a graph
def graph2(latency, title):
    fig, ax = plt.subplots(1, 1)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    plt.hist(latency, bins=200, label=label2(latency))
    plt.legend()
    plt.title("%s"%title, fontsize=15)
    plt.xlabel("Latency [us]", fontsize=15)
    plt.ylabel("Sample", fontsize=15)
    filename = dirname + "%s"%title+".pdf"
    plt.savefig(filename)

# %%
# get callback in XXX node
def cb(node):    
    path = path_tracelog
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    #node handle
    S = l.index('Nodes:\n')
    E = l.index(HEADER_PUBLISHERS_E)
    nodel = [line for line in l[S:E] if node in line]
    nodename= nodel[0].split()[4]
    nodeh= nodel[0].split()[0]

    S = l.index(HEADER_SUBSCRIBERS_S)
    E = l.index('Subscription objects:\n')
    subt = [line for line in l[S:E] if nodeh in line]
    stopic=[]
    for i in subt:
        stopic.append('/'+i.split(' /')[1].split()[0])
    print('sub:',stopic)
    
    S = l.index(HEADER_PUBLISHERS_S)
    E = l.index(HEADER_SUBSCRIBERS_E)
    pubt = [line for line in l[S:E] if nodeh in line]
    ptopic=[]
    for i in pubt:
        ptopic.append('/'+i.split(' /')[1].split()[0])
    print('pub:',ptopic)
    
    S = l.index('Services:\n')
    E = l.index('Clients:\n')
    sert = [line for line in l[S:E] if nodeh in line]
    sname=[]
    for i in sert:
        sname.append('/'+i.split(' /')[1].split()[0])
    #print('service:',sname)
    
    S = l.index('Clients:\n')
    E = l.index('Timers:\n')
    clit = [line for line in l[S:E] if nodeh in line]
    csname=[]
    for i in clit:
        csname.append('/'+i.split(' /')[1].split()[0])
    #print('client:',csname)

def cbname(node,topic):
    path = path_tracelog
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    
    #node handle
    S = l.index('Nodes:\n')
    E = l.index(HEADER_PUBLISHERS_E)
    node = [line for line in l[S:E] if node in line]
    nodename= node[0].split()[4]
    nodeh= node[0].split()[0]

    S = l.index(HEADER_SUBSCRIBERS_S)
    E = l.index('Subscription objects:\n')
    subt = [line for line in l[S:E] if nodeh in line]
    st=[line for line in subt if topic in line]
    #print(st)
    subscription_handle=st[0].split()[0]
    
    #subscription_handle -> reference
    S = l.index('Subscription objects:\n')
    E = l.index('Services:\n')   
    refer=[]
    ref = [line for line in l[S:E] if subscription_handle in line]
    refer.append(''.join(ref))
    reference= refer[0].split()[0]
    
    #reference -> callback_object
    S = l.index('Callback objects:\n')
    E = l.index('Callback symbols:\n')   
    cboj=[]
    cb= [line for line in l[S:E] if reference in line]
    cboj.append(''.join(cb))
    callback_object=cboj[0].split()[2]
    cbh = str(hex(int(callback_object)))
    cbhandle = cbh.upper().replace('X', 'x')

    return cbhandle

# get callback duration
def cbd(node, topic, title=''):
    path = path_tracelog
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    
    #node handle
    S = l.index('Nodes:\n')
    E = l.index(HEADER_PUBLISHERS_E)
    node = [line for line in l[S:E] if node in line]
    nodename= node[0].split()[4]
    nodeh= node[0].split()[0]
    S = l.index(HEADER_SUBSCRIBERS_S)
    E = l.index('Subscription objects:\n')
    subt = [line for line in l[S:E] if nodeh in line]
    st=[line for line in subt if topic in line]
    subscription_handle= st[0].split()[0]
    
    #subscription_handle -> reference
    S = l.index('Subscription objects:\n')
    E = l.index('Services:\n')   
    refer=[]
    ref = [line for line in l[S:E] if subscription_handle in line]
    refer.append(''.join(ref))
    reference= refer[0].split()[0]
    
    #reference -> callback_object
    S = l.index('Callback objects:\n')
    E = l.index('Callback symbols:\n')   
    cboj=[]
    cb= [line for line in l[S:E] if reference in line]
    cboj.append(''.join(cb))
    callback_object=cboj[0].split()[2]

    t=[]
    S = l.index('Callback instances:\n')
    E = l.index('Lifecycle state machines:\n')  
    l_XXX = [line for line in l[S:E] if callback_object in line]

    cbd_ns = []
    for i in l_XXX:
        cbd_ns.append(int(i.split()[6].split('.')[1])) 
    cbd_us = []
    for i in cbd_ns:
        cbd_us.append(i/1000)
    
    for i in cbd_us:
        if i > 200000:
            cbd_us.remove(i)
            
    _title = title if title else nodename

    return cbd_us,_title

# get callback duration
def t_cbd(node, title=''):
    path = path_tracelog
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    
    #node handle
    S = l.index('Nodes:\n')
    E = l.index(HEADER_PUBLISHERS_E)
    node = [line for line in l[S:E] if node in line]
    nodename= node[0].split()[4]
    nodeh= node[0].split()[0]
    S = l.index(HEADER_SUBSCRIBERS_S)
    E = l.index('Subscription objects:\n')
    # subt = [line for line in l[S:E] if nodeh in line]
    # st=[line for line in subt if topic in line]
    # subscription_handle= st[0].split()[0]
    
    #nodeh -> timer_handle
    S = l.index('Timer-node links:\n')
    E = l.index('Callback objects:\n')
    refer=[]
    ref = [line for line in l[S:E] if nodeh in line]
    refer.append(''.join(ref))
    reference= refer[0].split()[0]
    
    #reference -> callback_object
    S = l.index('Callback objects:\n')
    E = l.index('Callback symbols:\n')   
    cboj=[]
    cb= [line for line in l[S:E] if reference in line]
    cboj.append(''.join(cb))
    callback_object=cboj[0].split()[2]

    t=[]
    S = l.index('Callback instances:\n')
    E = l.index('Lifecycle state machines:\n')  
    l_XXX = [line for line in l[S:E] if callback_object in line]

    cbd_ns = []
    for i in l_XXX:
        cbd_ns.append(int(i.split()[6].split('.')[1])) 
    cbd_us = []
    for i in cbd_ns:
        cbd_us.append(i/1000)
    
    for i in cbd_us:
        if i > 200000:
            cbd_us.remove(i)
            
    _title = title if title else nodename

    return cbd_us,_title

def com_latency(callback1,callback2):
    path = path_end
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    
    com1_latency = []
    for i in l:
        cb1 = i.split("callback = ")[1].split(" }")[0]
        if (cb1 == callback1):
            com1 = int(i.split('+0.')[1].split(')')[0])
            com1_latency.append(com1)
            
    path = path_start
    f = open(path)
    f.close()
    with open(path) as f:
        l = f.readlines()
    
    com2_latency = []
    for i in l:
        cb2 = i.split("callback = ")[1].split(",")[0]
        if (cb2 == callback2):
            com2 = int(i.split('+0.')[1].split(')')[0])
            com2_latency.append(com2)
    
    com_latency=[com1_latency[i]+com2_latency[i] for i in range(min(len(com1_latency),len(com2_latency)))]

    com_latency_us = []
    for i in com_latency:
        com_latency_us.append(i/1000)
    return com_latency_us

# %%
node_gps = 'global_parameters_node'
node_ssm = 'system_status_manager_node'
node_msm = 'mission_manager_node'
node_odi = 'override_device_interface_node'
node_rci = 'rc_interface_node'
node_awl = 'aw_localization_node'
# node_geo = 'localization_geofence_node'
# node_res = 'localization_resilient_node'
# node_map = 'localization_globalmap_node'
node_wpt = 'waypoint_manager_node_mission_selective'
node_dyn = 'dynamic_planning_node'
node_vel = 'velocity_planning_node'
node_acc = 'nif_idm_based_acc_node'
node_lqr = 'control_joint_lqr'
node_csl = 'control_safety_layer_node'
node_acn = 'AccelControlNode'
# node_dbw = 'raptor_dbw_can_node'
# node_ssc = 'ssc_interface_node'
node_clu = 'lidar_clustering_node'
node_trc = 'objects_tracking_node'
node_cnc = 'perception_concat_node'
node_pre = 'opponent_predictor_node'
node_gff = 'geofence_filter_node'
node_urd = 'robot_state_publisher'

# %%
# cb(node_awl)
# cb(node_ssm)
# cb(node_msm)
# cb(node_odi)
# cb(node_rci)
# cb(node_geo)
# cb(node_res)
# cb(node_map)
# cb(node_wpt)
# cb(node_dyn)
# cb(node_vel)
# cb(node_acc)
# cb(node_lqr)
cb(node_csl)
# cb(node_acn)
# cb(node_clu)
# cb(node_dbw)
# cb(node_ssc)

# %%
topic_ss = '/system/status'
topic_sm = '/system/mission'
topic_ns_csl = '/node_status/control_safety_layer_node'
topic_ns_acc = '/node_status/nif_idm_based_acc_node'
topic_ns_vel = '/node_status/velocity_planning_node'
topic_ns_awl = '/node_status/aw_localization_node'
topic_ns_lqr = '/node_status/control_joint_lqr'
topic_ns_dyn = '/node_status/dynamic_planning_node'
topic_ns_wpt = '/node_status/waypoint_manager_node_mission_selective'
topic_ns_msm = '/node_status/mission_manager_node'
topic_ns_gff = '/node_status/geofence_filter_node'
# topic_ns_res = '/node_status/localization_resilient_node'

topic_veh_ejy = '/vehicle/emergency_joystick'
topic_veh_ehb = '/vehicle/emergency_heartbeat'

topic_joy_acc = '/joystick/accelerator_cmd'
topic_joy_brk = '/joystick/brake_cmd'
topic_joy_gea = '/joystick/gear_cmd'
topic_joy_str = '/joystick/steering_cmd'

topic_csl_out_cmd = '/control_safety_layer/out/control_cmd'
topic_csl_out_dac = '/control_safety_layer/out/desired_accel'

topic_cpool_ovr = '/control_pool/override_cmd'
topic_cpool_cmd = '/control_pool/control_cmd'

topic_acc_acc = '/control/acc/des_acc'
topic_vel_vel = '/velocity_planner/des_vel'
topic_awl_odo = '/aw_localization/ekf/odom'
topic_awl_err = '/aw_localization/ekf/error'
topic_awl_sts = '/aw_localization/ekf/status'

topic_perc_concat = '/perception/concat'
topic_perc_clu_pl = '/clustered/perception_list'
topic_perc_esr_track = '/radar_front/esr_track'

topic_rci_rfs = '/rc_interface/rc_flag_summary'
topic_pla_glo = '/planning/path_global'
topic_dyn_glo = '/planning/dynamic/traj_global'
topic_hb = '/diagnostics/heartbeat'
topic_clu_cp = '/cluster_center_points'
topic_tf = '/tf'

topic_sen_gnss_top = '/novatel_top/bestgnsspos'

# %%
tasks_cb = {
    "state_estimation" : [
        (node_awl,topic_sen_gnss_top),
    ],

    "perception" : [
        (node_clu, ),
        (node_trc, topic_perc_concat),
        (node_cnc, ),
        # (node_pre, ), # Missing data in the tracing session
        (node_gff, topic_perc_clu_pl),
    ],

    "planning" : [
        (node_wpt, ),
        (node_dyn, ),
        (node_vel, ),
    ],

    "control" : [
        (node_acc, ),
        (node_lqr, ),
        (node_csl, ),
        # (node_acn, topic_csl_out_dac),
    ]
}

# %%
cb1 = cbname(node_awl,topic_sen_gnss_top)
cb2 = cbname(node_dyn,topic_awl_odo)
cb3 = cbname(node_lqr,topic_dyn_glo)
cb4 = cbname(node_csl,topic_cpool_cmd)
cb5 = cbname(node_acn,topic_csl_out_dac)
# cb6 = cbname(node_ssc,topic_joy_acc)
# cb7 = cbname(node7,topic7)
# cb8 = cbname(node8,topic8)
# cb9 = cbname(node9,topic9)
# cb10 = cbname(node10,topic10)
# cb11 = cbname(node11,topic11)
# cb12 = cbname(node12,topic12)
# cb13 = cbname(node13,topic13)
# cb14 = cbname(node14,topic14)
# cb15 = cbname(node15,topic15)
# cb16 = cbname(node16,topic16)

# %%
import csv 

with open('tracing_out.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(
        ['task', 'node', 'topic', 'timer_handle', 'callback_duration_us']
    )
    for task in tasks_cb:
        for step in tasks_cb[task]:
            if len(step) >= 2:
                cbduration,_title = cbd(step[0], step[1])
            else:
                cbduration,_title = t_cbd(step[0])
            rows = [
                [task, step[0], step[1] if len(step) >= 2 else '', 'y' if len(step) < 2 else '', cbdur] for cbdur in cbduration
            ]
            writer.writerows(rows)
            
            title = task + ':' + _title
            len(cbduration)
            cbduration_ms=[i/1000 for i in cbduration]
            graph(cbduration_ms,title)
