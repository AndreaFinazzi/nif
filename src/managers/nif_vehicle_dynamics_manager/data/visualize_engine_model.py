import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import seaborn as sns


uniform_data = np.random.rand(10, 12)

"""
Visualize Engine model
- Engine model : (engine_speed, throttle)
    - engine_speed_min = 900
    - engine_speed_res = 50 
    - engine_speed_max = 7000
    - throttle_min = 0
    - throttle_res = 1
    - throttle_max = 100
"""

def calcIndex(input_data, data_min, data_res, data_max):
    data_length = int((data_max - data_min) / data_res)
    if input_data > data_max:
        return data_length - 1
    elif input_data < data_min:
        return 0
    
    return int((input_data - data_min) / data_res)

def main():
    dir_path = 'engine_model'
    file_name = 'engine_map_extrapolated_with_data_210930_132133.csv'
    file_path = dir_path + '/' + file_name

    df = pd.read_csv(file_path, header=None)
    engine_model = df.to_numpy()
    print("(engine_speed, throttle) :", df.shape)

    engine_speed_min = 900
    engine_speed_res = 50 
    engine_speed_max = 7000
    throttle_min = 0
    throttle_res = 1
    throttle_max = 100

    # # Heatmap
    # sns.set()
    # ax = sns.heatmap(engine_model)
    # ax.invert_yaxis()

    # Torque plot
    sample_RPM_list = np.array([900, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000, 6000, 7000])
    # sample_RPM_list = np.array([900, 950, 1000, 1050, 1100])
    # sample_RPM_list = np.array([1150, 1200, 1250, 1300, 1400, 1500, 1600, 2000])
    # sample_RPM_list = np.array([2000, 2200, 2400, 2600, 2800, 3000])
    # sample_RPM_list = np.array([2800, 3000, 3200, 3400, 3600, 3800, 4000])
    # sample_RPM_list = np.array([3300]) # noisy torque plot
    # sample_RPM_list = np.array([2500]) # 
 
    plt.figure()
    plt.grid()

    for RPM in sample_RPM_list:
        idx_RPM = calcIndex(RPM, engine_speed_min, engine_speed_res, engine_speed_max)
        torque_range = engine_model[idx_RPM, :]
        plt.plot(torque_range, '-')
    
    plt.title('Engine Map')
    plt.xlabel('Throttle')
    plt.ylabel('Engine Torque')
    plt.legend(sample_RPM_list)
    plt.show()


if __name__ == '__main__':
    main()