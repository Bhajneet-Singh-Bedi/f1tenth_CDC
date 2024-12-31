#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from queue import PriorityQueue

class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_following')
        
        self.ranges_list=[]
        self.bubble_radius=50
        self.blur_size = 10
        self.safe_distance = 2.0
        self.danger_distance = 1.0
        self.jump_threshold = 2.0
        self.max_velocity = 0.15 
        self.min_velocity = 0.07
        self.max_gap_size = 300
        self.min_gap_size = 50
        self.smoothing_window = 20

        
        self.lidar_sub = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.lidar_callback, 1)
        
        
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 1)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 1)

    def smoothed_scan(self, ranges_lst):
        """
        Gives 270 degrees ranges instead of every 0.25 degree.
        Resolution is 1 instead of .25
        """
        return [ranges_lst[i] for i in range(0, len(ranges_lst), 4)]

    def bubble_list(self, ranges_lst):
        """
        Puts bubble around min value out of the list.
        Those bubble contains 0 value.
        """
        min_idx = np.argmin(ranges_lst)
        if min_idx-self.bubble_radius < 0:
            start_idx=0
            end_idx=min_idx+self.bubble_radius
        elif min_idx+self.bubble_radius > len(ranges_lst)-1:
            start_idx=min_idx-self.bubble_radius
            end_idx=len(ranges_lst)-1
        else:
            start_idx=min_idx-self.bubble_radius
            end_idx=min_idx+self.bubble_radius

        for i in range(start_idx, end_idx+1):
            ranges_lst[i]=0
        return ranges_lst


    def max_gap(self, ranges_lst):
        """
        finds the max gap in the ranges_lst 
        and returns the max_start and max_end 
        index from the list.
        """
        max_count = 0  
        current_count = 0  
        start_index = -1  
        max_start = -1  
        max_end = -1  

        for i, num in enumerate(ranges_lst):
            if num != 0:
                if current_count == 0:
                    start_index = i
                current_count += 1
                if current_count > max_count:
                    max_count = current_count
                    max_start = start_index
                    max_end = i
            else:
                current_count = 0

        return max_start, max_end

    def best_heading(self, index, processed_list):
        """
        returns the best heading index where the f1 should go.
        """
        idx = index[0]
        safe_ranges = PriorityQueue()
        while idx < index[1]:
            if processed_list[idx] >= self.safe_distance:
                safe_start = idx
                idx += 1
                while (idx < index[1] and processed_list[idx] >= self.safe_distance
                       and idx - safe_start <= self.max_gap_size
                       and abs(processed_list[idx] - processed_list[max(0, idx - 1)]) < self.jump_threshold):
                    idx += 1
                safe_end = max(0, idx - 1)
                if safe_end != safe_start:
                    safe_ranges.put((-(np.max(processed_list[safe_start:safe_end])), (safe_start, safe_end)))
            else:
                idx += 1
        if safe_ranges.empty():
            print('No safe ranges found')
            return np.argmax(processed_list)
        else:
            while not safe_ranges.empty():
                safe_start, safe_end = safe_ranges.get()[1]
                target_idx = (safe_start + safe_end) // 2
                if 179 <= target_idx <= 900 and safe_end - safe_start > self.min_gap_size:
                    print(f'left: {safe_start}, right: {safe_end}')
                    return target_idx
            return target_idx

        #### Over shooting ####
        # return (index[0]+index[1]+1)/2
        #### Crashing ####
        # return np.argmax(processed_list[index[0]:index[1]+1])

    def lidar_callback(self, msg: LaserScan):
        """
        0 degree is forward,
        -135 is on 4th quadrant
        -90 on right x
        135 on 3rd quadrant.
        range goes from -135 to 0 to 135.
        resulution is 0.25, which means every 4th point gives one angle.
        0th pt  -->  -135, 3rd point  -->  -134 and so on.
        """

        # Gives 45 Degrees range from forward.
        # ranges_list = msg.ranges[630:450:-1]
        # farthest_pt_idx = np.argmax(ranges_list)

        ranges_list=msg.ranges
        # ranges_list = self.smoothed_scan(msg.ranges)
        processed_list = self.bubble_list(ranges_list)

        idx = self.max_gap(processed_list)
        # best_index = (idx[0]+idx[1]+1)/2
        best_index = self.best_heading(idx, processed_list)

        angle = msg.angle_min + best_index * msg.angle_increment
        
        print("Best Index:- ", best_index)
        print("angle:- ", angle)

        velocity=self.max_velocity
        if abs(angle)>0.5:
            velocity=self.min_velocity
        
        print("Velocity:- ", velocity)

        throttle_msg = Float32()
        throttle_msg.data = velocity
        self.throttle_pub.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = angle
        self.steering_pub.publish(steering_msg)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
