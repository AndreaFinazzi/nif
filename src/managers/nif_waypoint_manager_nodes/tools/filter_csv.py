import csv
with open('lgsvl_sim/wpt_left_filtered.csv', 'w', newline='') as csvfile_w:
    writer = csv.writer(csvfile_w, delimiter=' ',
                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
    with open('lgsvl_sim/wpt_left.csv', newline='') as csvfile_r:
        reader = csv.reader(csvfile_r, delimiter=',', quotechar='|')
        i = 0
        j = 0
        for row in reader:
            if i == 100:
                writer.writerow(row)
                last_row = row
                print(row)
                i = 0
            else:
                i += 1

                # if last_row[0] is not str():
                #     if (last_row[0] - row[0]) ** 2 + (last_row[1] - row[1]) ** 2 > 0.5 ** 2:
                #         writer.writerow(row)
                #         last_row = row