#!/usr/bin/env python3
"""
Simple bbox -> OccupancyGrid publisher.

Reads a YAML file containing a list of bounding-box polygons (each in some frame,
typically 'odom'), transforms them into a target frame (default 'map'), rasterizes
them into a small OccupancyGrid and publishes on a configurable topic (default
`/keepout_filter_mask`). This allows the existing `costmap_filter_info_server` to
consume the mask without using a map_server.

This is intentionally small and dependency free (only rclpy and tf2_ros).
"""
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

try:
    import tf2_ros
except Exception:
    tf2_ros = None


def point_in_poly(x, y, poly):
    """Ray casting algorithm for point-in-polygon test."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside


class BBoxMaskServer(Node):
    def __init__(self, yaml_file: str):
        super().__init__('bbox_mask_server')
        
        # Declare and get use_sim_time parameter (may already be set by command line)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'use_sim_time: {use_sim_time}')

        if tf2_ros is None:
            self.get_logger().error('tf2_ros is required but not available')
            raise RuntimeError('tf2_ros not available')

        if not os.path.isabs(yaml_file):
            # allow relative path from package root
            yaml_file = os.path.abspath(yaml_file)

        if not os.path.exists(yaml_file):
            self.get_logger().error(f'YAML file not found: {yaml_file}')
            raise RuntimeError('yaml not found')

        with open(yaml_file, 'r') as f:
            doc = yaml.safe_load(f)

        # params with defaults
        self.bboxes = doc.get('bboxes', [])
        self.resolution = float(doc.get('resolution', 0.1))
        self.topic = doc.get('topic', '/keepout_filter_mask')
        self.publish_rate = float(doc.get('publish_rate', 1.0))
        self.target_frame = doc.get('target_frame', 'map')
        self.max_cells = int(doc.get('max_cells', 1000 * 1000))
        self.tf_wait_time = float(doc.get('tf_wait_time', 5.0))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, self.topic, qos)

        self.get_logger().info(f'Loaded {len(self.bboxes)} bboxes; will publish to {self.topic} in frame {self.target_frame}')
        self.get_logger().info(f'Waiting {self.tf_wait_time}s for TF to be ready...')
        
        # Cache the generated mask
        self.cached_mask = None
        self.tf_ready = False
        
        # Start with a one-shot timer to wait for TF, then switch to regular publishing
        self.startup_timer = self.create_timer(self.tf_wait_time, self.initial_publish_cb)
        self.publish_timer = None

    def transform_point(self, x, y, src_frame, t):
        # apply stamp t (TransformStamped) to point
        # t is transform from src_frame -> target_frame
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        # rotation by quaternion (z only assumed?) do full 2D rotation
        # compute yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # rotate and translate
        xr = x * math.cos(yaw) - y * math.sin(yaw) + tx
        yr = x * math.sin(yaw) + y * math.cos(yaw) + ty
        return xr, yr

    def initial_publish_cb(self):
        """Called once after startup delay to generate and publish the mask."""
        self.startup_timer.cancel()
        self.get_logger().info('Initial publish callback - generating mask...')
        
        if self.generate_mask():
            self.get_logger().info('Mask generated successfully. Starting periodic publishing...')
            # Start regular publishing timer
            self.publish_timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self.timer_cb)
            self.tf_ready = True
        else:
            self.get_logger().error('Failed to generate mask. Will retry...')
            # Retry after another delay
            self.startup_timer = self.create_timer(2.0, self.initial_publish_cb)

    def generate_mask(self):
        """Generate the occupancy grid mask from bboxes. Returns True on success."""
        if not self.bboxes:
            self.get_logger().warn('No bboxes configured')
            return False

        # transform all bbox points into target frame
        all_polys = []
        minx = float('inf')
        miny = float('inf')
        maxx = float('-inf')
        maxy = float('-inf')

        for box in self.bboxes:
            frame = box.get('frame', 'odom')
            corners = box.get('corners', [])
            if len(corners) < 3:
                self.get_logger().warn(f'Skipping bbox with < 3 corners')
                continue

            # look up transform from frame -> target_frame
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    frame, 
                    rclpy.time.Time(), 
                    timeout=Duration(seconds=2.0)
                )
            except Exception as e:
                self.get_logger().error(f'Failed to get transform {frame} -> {self.target_frame}: {e}')
                return False

            poly = []
            for p in corners:
                x, y = float(p[0]), float(p[1])
                xr, yr = self.transform_point(x, y, frame, trans)
                poly.append((xr, yr))
                minx = min(minx, xr)
                miny = min(miny, yr)
                maxx = max(maxx, xr)
                maxy = max(maxy, yr)

            all_polys.append(poly)
            self.get_logger().info(f'Transformed bbox from {frame}: {len(corners)} corners')

        if not all_polys:
            self.get_logger().error('No valid bboxes to rasterize')
            return False

        width = int(math.ceil((maxx - minx) / self.resolution))
        height = int(math.ceil((maxy - miny) / self.resolution))

        if width * height > self.max_cells:
            self.get_logger().error(f'Grid too large: {width}x{height} cells ({width*height} > {self.max_cells})')
            return False

        self.get_logger().info(f'Generating {width}x{height} grid (bounds: [{minx:.2f}, {miny:.2f}] to [{maxx:.2f}, {maxy:.2f}])')

        grid = [0] * (width * height)

        # rasterize polygons (no padding - use Nav2 InflationLayer for obstacle inflation)
        occupied_count = 0
        for iy in range(height):
            y = miny + (iy + 0.5) * self.resolution
            row_off = iy * width
            for ix in range(width):
                x = minx + (ix + 0.5) * self.resolution
                for poly in all_polys:
                    if point_in_poly(x, y, poly):
                        grid[row_off + ix] = 100
                        occupied_count += 1
                        break

        self.get_logger().info(f'Rasterized {occupied_count} occupied cells out of {width*height}')

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        meta = MapMetaData()
        meta.resolution = float(self.resolution)
        meta.width = width
        meta.height = height
        meta.origin.position.x = float(minx)
        meta.origin.position.y = float(miny)
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0

        msg.info = meta
        msg.data = grid

        self.cached_mask = msg
        return True

    def timer_cb(self):
        """Periodic callback to republish the cached mask."""
        if self.cached_mask is None:
            self.get_logger().warn_once('No cached mask to publish')
            return

        # Update timestamp
        self.cached_mask.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cached_mask)
        self.get_logger().debug(f'Published keepout mask to {self.topic}')


def main(argv=None):
    rclpy.init(args=argv)
    if argv is None:
        argv = sys.argv

    if len(argv) < 2:
        print('Usage: bbox_mask_server.py <keepout_bboxes.yaml>')
        return 1

    yaml_file = argv[1]
    try:
        node = BBoxMaskServer(yaml_file)
        rclpy.spin(node)
    except Exception as e:
        print('Exception:', e)
        return 2
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
