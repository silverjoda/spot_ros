import rospy
import bosdyn.client.local_grid
import bosdyn.client
from bosdyn.client import frame_helpers
import numpy as np
import yaml
import struct
import sys

import tf2_ros
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

class LocalGridPublisher:
    def __init__(self):
        self.init_spot_com()
        self.init_ros()

    def init_spot_com(self):
        self.sdk = bosdyn.client.create_standard_sdk('local-grid-publisher')
        self.robot = self.sdk.create_robot('192.168.50.3')
        self.robot.authenticate('robot', 'niftiniftinifti')
        self.grid_client = self.robot.ensure_client(bosdyn.client.local_grid.LocalGridClient.default_service_name)
        # self.grid_names = ["terrain", "terrain_valid", "intensity", "no_step", "obstacle_distance"]
        self.grid_names = ["terrain", "terrain_valid"]
        self.format_map = {0: "@", 1: "<f", 2: "<d", 3: "<b", 4:"<B", 5:"<h", 6: "<H"}

    def init_ros(self):
        self.node = rospy.init_node("spot_grid_perception")
        self.rate = rospy.Rate(5)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.gc_publisher_dict = {}
        for grid_name in self.grid_names:
            self.gc_publisher_dict[grid_name] = rospy.Publisher(f"spot/grid/{grid_name}", GridCells, queue_size=1)

    def decode(self, data, cell_format):
        n_b = struct.calcsize(self.format_map[cell_format])
        data_arr = []
        for i in range(0, len(data), n_b):
            dec_pt = struct.unpack(self.format_map[cell_format], data[i:i+n_b])
            data_arr.append(dec_pt[0])
        return data_arr

    def get_local_grids(self, names):
        # Ros grid messages
        grid_msgs = {}

        # Get all grids in protobuff format
        grids = self.grid_client.get_local_grids(names)

        for name, grid in zip(names, grids):
            # Get all available fields info out of the protobuff msg
            local_grid_type_name = grid.local_grid.local_grid_type_name
            acquisition_time = grid.local_grid.acquisition_time
            transforms_snapshot = grid.local_grid.transforms_snapshot
            frame_name_local_grid_data = grid.local_grid.frame_name_local_grid_data
            extent = grid.local_grid.extent # cell_size, num_cells_x, num_cells_y
            cell_format = grid.local_grid.cell_format
            encoding = grid.local_grid.encoding # 0: unspecified, 1: raw, 2: rle
            data = grid.local_grid.data
            rle_counts = grid.local_grid.rle_counts
            cell_value_scale = grid.local_grid.cell_value_scale
            cell_value_offset = grid.local_grid.cell_value_offset
            data_comp = self.decode(data, cell_format)
            
            if encoding == 2:
                data_dec = []
                # Decode using rle
                for i in range(len(rle_counts)):
                    for _ in range(rle_counts[i]):
                        data_dec.append(data_comp[i])
            else:
                data_dec = data_comp
            
            #Check that we have the correct name
            assert local_grid_type_name == name

            # Calculate transform from grid corner to body frame
            grid_to_body_tf = frame_helpers.get_a_tform_b(transforms_snapshot, "body", frame_name_local_grid_data)
            
            # Decode the data
            points = []
            idx = 0
            for i, d in enumerate(data_dec):
                # Height of cell
                z = d * cell_value_scale + cell_value_offset

                # x and y values (in grid frame)
                x = (idx % extent.num_cells_x) * extent.cell_size
                y = (idx // extent.num_cells_x) * extent.cell_size
                idx += 1

                # Apply transform (just translation here)
                x_tf = grid_to_body_tf.position.x
                y_tf = grid_to_body_tf.position.y
                z_tf = grid_to_body_tf.position.z

                # Add point to points list
                pt = Point()
                pt.x = x_tf + x
                pt.y = y_tf + y
                pt.z = z_tf + z
                
                points.append(pt)

            # Make ros message
            grid_msg = GridCells()
            grid_msg.header.frame_id = "body"
            grid_msg.header.stamp = rospy.Time(acquisition_time.seconds, acquisition_time.nanos)
            grid_msg.cells = points
            grid_msg.cell_width = extent.cell_size
            grid_msg.cell_height = extent.cell_size

            grid_msgs[name] = grid_msg

        return grid_msgs

    def publish_msgs(self, cell_msg_dict):
        for name, msg in cell_msg_dict.items():
            self.gc_publisher_dict[name].publish(msg)

    def loop(self):
        while not rospy.is_shutdown():

            cell_msg_dict = self.get_local_grids(self.grid_names)
            self.publish_msgs(cell_msg_dict)

            self.rate.sleep()

if __name__ == '__main__':
    local_grid_publisher = LocalGridPublisher()
    local_grid_publisher.loop()
