import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import csv
import argparse
import os
from os.path import isfile, join
from itertools import zip_longest
import numpy as np
from visualization_msgs.msg import MarkerArray


class BagFileParser():
    def __init__(self, bag_file_path):
        self.conn = sqlite3.connect(bag_file_path)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data if type_of == 'visualization_msgs/msg/MarkerArray'}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag-directory")
    parser.add_argument("-o", "--output-path")
    parser.add_argument("-n", "--num-of-ai", type=int, default=1)
    parser.add_argument("-p", "--ai-array-topic", type=str, default="/ac/vis/cars")
    parser.add_argument("-v", "--verbose", default=True)

    args = parser.parse_args()

    bag_directory = args.bag_directory
    output_path = args.output_path
    # num_of_ai = args.num_of_ai
    ai_array_topic = args.ai_array_topic
    verbose = args.verbose

    bag_directory = os.path.realpath(bag_directory)
    assert os.path.isdir(bag_directory)

    # Create the output path if it doesnt exist
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    db3_file_list = [
            f for f in os.listdir(bag_directory)
            if isfile(join(bag_directory, f)) and "db3" in f
        ]

    # ai_msgs_topic_name_list = []
    # ai_msgs_list = []
    # for ai_idx in range(num_of_ai):
    #     # ai_msgs_topic = ai_topic_prefix + "/" + ai_idx
    #     ai_msgs_topic = ai_topic_prefix
    #     if(verbose):
    #         print("Collect AI #", ai_idx, "with topic name : ", ai_msgs_topic)
    #     ai_msgs_topic_name_list.append(ai_msgs_topic)

    db3_idx = 0
    for db3_file in db3_file_list:
        db3_file_path = os.path.realpath(bag_directory + "/" + db3_file)
        db3_idx = db3_idx + 1
        parser = BagFileParser(db3_file_path)

        filename = (
                os.path.join(
                    output_path, "AC_bag_" + "{:08d}".format(db3_idx) + ".csv"
                )
            )

        write_target = []

        with open(filename, mode='w') as file:
            ac_data_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            ai_array_msgs = parser.get_messages(ai_array_topic)

            assert len(ai_array_msgs)
            num_of_ai = len(ai_array_msgs[0][1].markers)

            ai_array_pos_x = []
            ai_array_pos_y = []

            for ai_idx in range(num_of_ai):
                ai_pos_x = [ai_array_msgs[i][1].markers[ai_idx].pose.position.x for i in range(len(ai_array_msgs)) if ai_array_msgs[i][1].markers[ai_idx].id == ai_idx]
                ai_pos_y = [ai_array_msgs[i][1].markers[ai_idx].pose.position.y for i in range(len(ai_array_msgs)) if ai_array_msgs[i][1].markers[ai_idx].id == ai_idx]
                ai_pos_x_diff = np.diff(np.array(ai_pos_x))
                ai_pos_y_diff = np.diff(np.array(ai_pos_y))
                ai_yaw_rad = list(np.arctan2(ai_pos_y_diff, ai_pos_x_diff))

                ai_pos_x.insert(0, "AI_" + str(ai_idx) + "_pos_x")
                ai_pos_y.insert(0, "AI_" + str(ai_idx) + "_pos_y")
                ai_yaw_rad.insert(0, "AI_" + str(ai_idx) + "_yaw_rad")

                write_target.append(ai_pos_x[1:])
                write_target.append(ai_pos_y[1:])
                write_target.append(ai_yaw_rad)

            ac_data_writer.writerows(zip_longest(*write_target, fillvalue=''))  # write to csv
