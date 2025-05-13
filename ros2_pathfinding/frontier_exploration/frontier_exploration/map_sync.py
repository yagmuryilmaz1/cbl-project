#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np
from pathlib import Path

class MapSync(Node):
    def __init__(self):
        super().__init__('map_sync')
        
        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Create publishers
        self.map_status_pub = self.create_publisher(
            String,
            '/map_sync/status',
            10
        )
        
        # Get package directory
        self.pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.maps_dir = os.path.join(self.pkg_dir, 'maps')
        
        # Create maps directory if it doesn't exist
        if not os.path.exists(self.maps_dir):
            os.makedirs(self.maps_dir)
            
        self.get_logger().info('Map synchronization node initialized')
        
    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for map updates.
        Saves the map to a file and creates a YAML file with map metadata.
        """
        try:
            # Convert occupancy grid to numpy array
            map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            
            # Save map as PGM file
            map_path = os.path.join(self.maps_dir, 'map.pgm')
            self.save_pgm(map_path, map_data)
            
            # Create YAML file with map metadata
            yaml_path = os.path.join(self.maps_dir, 'map.yaml')
            self.save_yaml(yaml_path, msg.info, 'map.pgm')
            
            # Publish success status
            status_msg = String()
            status_msg.data = f'Map saved successfully to {map_path}'
            self.map_status_pub.publish(status_msg)
            self.get_logger().info(status_msg.data)
            
        except Exception as e:
            error_msg = f'Error saving map: {str(e)}'
            self.get_logger().error(error_msg)
            status_msg = String()
            status_msg.data = error_msg
            self.map_status_pub.publish(status_msg)
    
    def save_pgm(self, filepath: str, data: np.ndarray):
        """
        Save occupancy grid data as a PGM file.
        """
        # Convert occupancy grid values to PGM format
        # -1 (unknown) -> 205
        # 0 (free) -> 254
        # 100 (occupied) -> 0
        pgm_data = np.zeros_like(data, dtype=np.uint8)
        pgm_data[data == -1] = 205
        pgm_data[data == 0] = 254
        pgm_data[data == 100] = 0
        
        # Write PGM file
        with open(filepath, 'wb') as f:
            f.write(b'P5\n')
            f.write(f'{data.shape[1]} {data.shape[0]}\n'.encode())
            f.write(b'255\n')
            f.write(pgm_data.tobytes())
    
    def save_yaml(self, filepath: str, map_info, image: str):
        """
        Save map metadata as a YAML file.
        """
        yaml_data = {
            'image': image,
            'resolution': map_info.resolution,
            'origin': [
                map_info.origin.position.x,
                map_info.origin.position.y,
                map_info.origin.position.z
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    node = MapSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 