import numpy
import csv
import matplotlib.pyplot as plt

terrain_data_file_path_list = [
    "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_inner_path_dataLog_partial_error.csv",
    "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_inner_path_dataLog_partial_error_compensate.csv",
    # "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_middle_path_dataLog_v1.csv",
    # "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_middle_path_dataLog_v2.csv",
    # "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_outter_path_dataLog_v2.csv"
]
# "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/terrain_data/VehiclePlayerExport_outter_path_dataLog.csv",

terrain_x_total = []
terrain_y_total = []
terrain_z_total = []

for filePath in terrain_data_file_path_list:
    terrain_x = []
    terrain_y = []
    terrain_z = []
    with open(filePath, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=';')
        for row_idx, row in enumerate(spamreader):
            if(row_idx != 0 and row_idx != 1):
                time = row[0]
                x = row[1]
                y = row[2]
                z = row[3]
                terrain_x.append(float(x))
                terrain_y.append(float(y))
                terrain_z.append(float(z))
    terrain_x_total.append(terrain_x)
    terrain_y_total.append(terrain_y)
    terrain_z_total.append(terrain_z)

fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.scatter(terrain_x, terrain_y, terrain_z)
# plt.scatter(terrain_x, terrain_y, s=1)
plt.axis('equal')
colors = ['salmon',  'steelblue']

test_idx = 14000
plt.scatter(terrain_x_total[0][test_idx],
            terrain_y_total[0][test_idx], s=20, c=colors[0])

test_idx = 15400
plt.scatter(terrain_x_total[0][test_idx],
            terrain_y_total[0][test_idx], s=20, c=colors[1])

test_idx = 0
plt.scatter(terrain_x_total[0][test_idx],
            terrain_y_total[0][test_idx], s=20, c=colors[0])


for i in range(len(terrain_x_total)):
    plt.scatter(terrain_x_total[i], terrain_y_total[i], s=1, c=colors[i])

plt.show()
