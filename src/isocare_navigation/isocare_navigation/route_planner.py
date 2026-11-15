"""route_planner: plans a route over the marker graph.

It moves data between ROS and route_core.py and nothing else.

ROS side:
    service  /plan_route    isocare_interfaces/PlanRoute
Parameter:
    marker_file   path of the marker table YAML (the launch file passes the one in
                  isocare_bringup/config). The node refuses to start if the file is not usable.
"""

from isocare_interfaces.srv import PlanRoute
from isocare_navigation import route_core
import rclpy
from rclpy.node import Node


class RoutePlanner(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Load and check the marker table, then offer the service."""
        super().__init__('route_planner')
        self.declare_parameter('marker_file', '')
        path = self.get_parameter('marker_file').value
        if not path:
            raise RuntimeError('parameter marker_file is not set')
        self.map = route_core.MarkerMap.load_file(path)       # raises with a clear message
        self.get_logger().info(
            f'loaded {len(self.map.markers)} markers and {self.map.edge_count()} edges '
            f'from {path}')
        self.create_service(PlanRoute, '/plan_route', self._on_request)

    def _on_request(self, request, response):
        """Answer one /plan_route request."""
        response.success = False
        response.names, response.marker_ids = [], []
        response.leg_distance_m, response.leg_heading_rad = [], []
        response.total_m = 0.0
        if len(request.blocked_from) != len(request.blocked_to):
            response.reason = 'blocked_from and blocked_to must have the same length'
            return response
        try:
            route = self.map.plan(request.from_name, request.to_name,
                                  list(zip(request.blocked_from, request.blocked_to)))
        except route_core.RouteError as error:
            response.reason = str(error)
            self.get_logger().warn(f'no route: {error}')
            return response
        response.success = True
        response.reason = ''
        response.names = list(route.names)
        response.marker_ids = list(route.marker_ids)
        response.leg_distance_m = [leg.distance_m for leg in route.legs]
        response.leg_heading_rad = [leg.heading_rad for leg in route.legs]
        response.total_m = route.total_m
        return response


def main(args=None):
    """Run the route planner until Ctrl-C."""
    rclpy.init(args=args)
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
