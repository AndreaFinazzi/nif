import numpy
import csv
import matplotlib.pyplot as plt

terrain_data_file_path_list = [
    "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/test_m_terrain_bankAngle_map.csv"]
# "/home/calvin/workspace/iac_hackathon_3/iac_terrain_manager/test_m_terrain_z_map.csv"]


terrain_x_total = []
terrain_y_total = []
terrain_z_total = []

for filePath in terrain_data_file_path_list:
    terrain_x = []
    terrain_y = []
    terrain_z = []
    cnt = 0
    with open(filePath, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',')
        for row_idx, row in enumerate(spamreader):
            for i in range(len(row)):
                # x = (i - len(row)/2) * 10
                # y = (row_idx - 77/2) * 10
                x = row_idx * 10 - 322
                y = i*10 - 2422
                z = row[i]

                if(x != "" and y != "" and z != ""):
                    if(i > 1 and i < len(row)):
                        terrain_x.append(float(x))
                        terrain_y.append(float(y))
                        if(float(z) > 0.017 and (float(row[i-1]) < 0.017 and float(row[i+1]) < 0.017)):
                            z = min(row[i-1], row[i+1])
                        terrain_z.append(min(float(z), 0.157))

                    # terrain_z.append((float(z)))
    terrain_x_total.append(terrain_x)
    terrain_y_total.append(terrain_y)
    terrain_z_total.append(terrain_z)

fig = plt.figure()
ax = fig.gca(projection='3d')
# print(len(terrain_x), len(terrain_y), len(terrain_z))
ax.scatter(terrain_x, terrain_y, terrain_z)
# plt.scatter(terrain_x, terrain_y)
# plt.scatter(terrain_x, terrain_y, s=1)
# plt.axis('equal')
# colors = ['salmon', 'orange', 'steelblue']

# for i in range(len(terrain_x_total)):
#     plt.scatter(terrain_x_total[i], terrain_y_total[i], s=1, c=colors[i])

plt.show()
