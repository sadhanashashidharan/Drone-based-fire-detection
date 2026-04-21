# scripts/patrol.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import Bool
import math

# Camera faces +X on the drone body.
# So to see a cube HEAD-ON, drone must sit at (cube_x - OFFSET, cube_y)
# facing +X. This works for ALL three cubes.

OFFSET = 2.5   # metres away from the cube face

class Patrol(Node):
    def __init__(self):
        super().__init__('patrol')

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo service...')

        # fire1=(8,0)   → drone at (8-OFFSET, 0)   facing +X
        # fire2=(-8,0)  → drone at (-8-OFFSET, 0)  facing +X  (approaches from left)
        # fire3=(0,8)   → drone at (-OFFSET, 8)     facing +X  (cube is to the right of path, approach along X)
        # All waypoints have drone facing +X so camera always looks straight at cube
        self.points = [
            ( 0.0,           0.0,  1.5),  # home
            ( 8.0 - OFFSET,  0.0,  1.5),  # face fire1 head-on
            ( 0.0,           0.0,  1.5),  # home
            (-8.0 - OFFSET,  0.0,  1.5),  # face fire2 head-on (drone left of cube)
            ( 0.0,           0.0,  1.5),  # home
            (-OFFSET,        8.0,  1.5),  # face fire3 head-on (cube is at x=0,y=8, drone at x=-2.5,y=8)
            ( 0.0,           0.0,  1.5),  # home
        ]

        # Fixed yaw = facing +X always (qz=0, qw=1)
        self.fixed_qz = 0.0
        self.fixed_qw = 1.0

        self.wp_index  = 0
        self.pos       = list(self.points[0])
        self.step_size = 0.2
        self.tick_rate = 0.05

        self.create_subscription(Bool, '/fire_detected', self.fire_cb, 10)
        self.timer = self.create_timer(self.tick_rate, self.tick)
        self.get_logger().info('Patrol started — camera faces every cube head-on!')

    def fire_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn(
                f'FIRE DETECTED at drone pos=({self.pos[0]:.1f},{self.pos[1]:.1f})')

    def _send(self, x, y, z):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'drone'
        req.state.pose.position.x    = float(x)
        req.state.pose.position.y    = float(y)
        req.state.pose.position.z    = float(z)
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = self.fixed_qz
        req.state.pose.orientation.w = self.fixed_qw
        self.client.call_async(req)

    def tick(self):
        tx, ty, tz = self.points[self.wp_index]
        dx   = tx - self.pos[0]
        dy   = ty - self.pos[1]
        dz   = tz - self.pos[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < self.step_size:
            self.pos = [tx, ty, tz]
            prev = self.wp_index
            self.wp_index = (self.wp_index + 1) % len(self.points)
            self.get_logger().info(f'Waypoint {prev} done -> next {self.wp_index}')
        else:
            s = self.step_size / dist
            self.pos[0] += dx * s
            self.pos[1] += dy * s
            self.pos[2] += dz * s

        self._send(*self.pos)


def main(args=None):
    rclpy.init(args=args)
    node = Patrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
