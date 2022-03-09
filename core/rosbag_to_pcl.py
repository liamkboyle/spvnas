from tokenize import Double
import numpy as np
import rosbag
import yaml
import sensor_msgs.msg
import sensor_msgs.point_cloud2
import torch


class RosbagToPCLExtractor:

    def __init__(self, rosbag_file, topic):
        self.rosbag_file = rosbag_file
        self.topic = topic
        self.scans = []
        print(rosbag_file)
        print("Loading rosbag " + self.rosbag_file + "...")
        self.bag = rosbag.Bag(self.rosbag_file)
        print("...done.")

        # Print information and check rosbag -----
        self.num_samples = 0
        info_dict = yaml.load(self.bag._get_yaml_info())
        print("Duration of the bag: " + str(info_dict["duration"]))
        for topic_messages in info_dict["topics"]:
            print(topic_messages['topic'])
            if topic_messages["topic"] == self.topic:
                self.num_samples = topic_messages["messages"]
        if self.num_samples > 0:
            print("Number of messages for topic " + self.topic + ": " + str(self.num_samples))
        else:
            print(self.topic)
            raise Exception("Topic " + self.topic + " is not present in the given rosbag (" + self.rosbag_file + ").")
        # -----------------------------------------
        for index, (topic, msg, t) in enumerate(self.bag.read_messages(topics=[self.topic])):
            if not index % 10:
                print("Preprocessing scan " + str(
                    index) + "/" + str(self.num_samples) + " from the point cloud " + self.rosbag_file + ".")
            if index == 100:
                break
            scan = self.ros_to_pcl(msg)
            self.scans.append(scan)

            # Write each scan to a .bin file
            filename = str(index) + ".bin"
            filename = filename.zfill(8)
            filename_path = rosbag_file.replace('ros/mapping.bag', "bin/" + filename)
            print('saving scan to: ', filename_path)
            scan.astype('float32').tofile(filename_path)

            
        self.scans = np.array(self.scans)


    def ros_to_pcl(self, ros_cloud):
        points_list = []
        for data in sensor_msgs.point_cloud2.read_points(ros_cloud, skip_nans=True):
            if(data[0] != 0 and data[1] != 0 and data[2] != 0):
                points_list.append([data[0], data[1], data[2], data[3]])
        points_list = np.asarray(points_list, dtype=np.float32)

        return points_list
    
    def get_scans(self):
        return self.scans
