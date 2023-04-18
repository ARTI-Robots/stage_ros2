import math
import random
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Client, Node
from rclpy.task import Future
from stage_ros2_itfs.action import MoveModel
from stage_ros2_itfs.srv import CreateModel, RemoveModels
from std_msgs.msg import ColorRGBA, Header


def create_quaternion_from_yaw_deg(yaw_deg: float = None):
    return Quaternion(x=0.0, y=0.0,
                      z=2 * math.sin(math.radians(yaw_deg) / 2),
                      w=2 * math.cos(math.radians(yaw_deg) / 2))


def get_name_of_class_constant(cls: type, prefix: str, value, default=None):
    for k, v in cls.__dict__.items():
        if k.startswith(prefix) and isinstance(v, type(value)) and v == value:
            return k[len(prefix):]  # return name with prefix removed
    return default


class ConveyedItem:
    INITIAL_POSE = Pose(position=Point(x=2.0, y=14.5, z=0.2))
    FINAL_POSE = Pose(position=Point(x=2.0, y=0.5, z=0.2))
    SIZE = Vector3(x=0.6, y=0.6, z=0.5)
    COLOR = ColorRGBA(r=0.8, g=0.8, b=0.5, a=1.0)

    def __init__(self, node: Node, i: int, create_model_client: Client,
                 move_model_client: ActionClient):
        future_response = create_model_client.call_async(CreateModel.Request(
            id=f'conveyed_item_{i}', pose=self.INITIAL_POSE, size=self.SIZE, color=self.COLOR))
        rclpy.spin_until_future_complete(node, future_response)
        response = future_response.result()  # type: CreateModel.Response
        if response.status == CreateModel.Response.STATUS_SUCCESS:
            self.id = response.id
            dx = random.uniform(-0.08, 0.08)
            dy = random.uniform(-0.05, 0.05)
            da = random.uniform(-3.0, 3.0)
            initial_pose = Pose(position=Point(x=self.INITIAL_POSE.position.x + dx,
                                               y=self.INITIAL_POSE.position.y + dy,
                                               z=self.INITIAL_POSE.position.z),
                                orientation=create_quaternion_from_yaw_deg(da))
            final_pose = Pose(position=Point(x=self.FINAL_POSE.position.x + dx,
                                             y=self.FINAL_POSE.position.y + dy,
                                             z=self.FINAL_POSE.position.z),
                              orientation=create_quaternion_from_yaw_deg(da))
            future_response = move_model_client.send_goal_async(MoveModel.Goal(
                id=response.id, trajectory=[
                    PoseStamped(header=Header(stamp=Time(sec=i)), pose=initial_pose),
                    PoseStamped(header=Header(stamp=Time(sec=i + 15)), pose=final_pose),
                ], iterations=MoveModel.Goal.ITERATIONS_INFINITE))
            rclpy.spin_until_future_complete(node, future_response)
            self.move_model_goal_handle = future_response.result()  # type: ClientGoalHandle

    def stop(self, node: Node):
        self.move_model_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, self.move_model_goal_handle.get_result_async())


class SlidingDoor:
    def __init__(self, node: Node, id_: str, closed_pose: Pose, open_pose: Pose, opening_time: int,
                 closing_time: int, total_time: int, move_model_client: ActionClient):
        self.logger = node.get_logger().get_child('sliding_door')
        self.id = id_
        self.open = False
        future_response = move_model_client.send_goal_async(MoveModel.Goal(
            id=self.id, trajectory=[
                PoseStamped(header=Header(stamp=Time(sec=0)), pose=closed_pose),
                PoseStamped(header=Header(stamp=Time(sec=opening_time)), pose=closed_pose),
                PoseStamped(header=Header(stamp=Time(sec=opening_time + 1)), pose=open_pose),
                PoseStamped(header=Header(stamp=Time(sec=closing_time)), pose=open_pose),
                PoseStamped(header=Header(stamp=Time(sec=closing_time + 1)), pose=closed_pose),
                PoseStamped(header=Header(stamp=Time(sec=total_time)), pose=closed_pose),
            ], iterations=MoveModel.Goal.ITERATIONS_INFINITE), self.process_feedback)
        rclpy.spin_until_future_complete(node, future_response)
        self.move_model_goal_handle = future_response.result()  # type: ClientGoalHandle

    def stop(self, node: Node):
        self.move_model_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, self.move_model_goal_handle.get_result_async())

    def process_feedback(self, feedback_msg: MoveModel.Impl.FeedbackMessage):
        if feedback_msg.feedback.trajectory_index == 2 and not self.open:
            self.open = True
            self.logger.info(f'door "{self.id}" has opened')
        elif feedback_msg.feedback.trajectory_index == 4 and self.open:
            self.open = False
            self.logger.info(f'door "{self.id}" has closed')


class Forklift:
    P0 = Pose(position=Point(x=15.0, y=12.5), orientation=create_quaternion_from_yaw_deg(180))
    P1 = Pose(position=Point(x=7.0, y=12.5), orientation=create_quaternion_from_yaw_deg(180))
    P2 = Pose(position=Point(x=6.0, y=11.5), orientation=create_quaternion_from_yaw_deg(-90))
    P3 = Pose(position=Point(x=6.0, y=3.5), orientation=create_quaternion_from_yaw_deg(-90))
    P4 = Pose(position=Point(x=7.0, y=2.5), orientation=create_quaternion_from_yaw_deg(0))
    P5 = Pose(position=Point(x=15.0, y=2.5), orientation=create_quaternion_from_yaw_deg(0))
    P6 = Pose(position=Point(x=16.0, y=3.5), orientation=create_quaternion_from_yaw_deg(90))
    P7 = Pose(position=Point(x=16.0, y=11.5), orientation=create_quaternion_from_yaw_deg(90))

    def __init__(self, node: Node, id_: str, move_model_client: ActionClient):
        self.id = id_
        self.logger = node.get_logger().get_child('forklift')
        self.laps = 0
        future_response = move_model_client.send_goal_async(MoveModel.Goal(
            id=self.id, trajectory=[
                PoseStamped(header=Header(stamp=Time(sec=1)), pose=self.P0),
                PoseStamped(header=Header(stamp=Time(sec=11)), pose=self.P1),
                PoseStamped(header=Header(stamp=Time(sec=13)), pose=self.P2),
                PoseStamped(header=Header(stamp=Time(sec=23)), pose=self.P3),
                PoseStamped(header=Header(stamp=Time(sec=25)), pose=self.P4),
                PoseStamped(header=Header(stamp=Time(sec=35)), pose=self.P5),
                PoseStamped(header=Header(stamp=Time(sec=37)), pose=self.P6),
                PoseStamped(header=Header(stamp=Time(sec=47)), pose=self.P7),
                PoseStamped(header=Header(stamp=Time(sec=49)), pose=self.P0),
            ], iterations=MoveModel.Goal.ITERATIONS_INFINITE, collision_mode=MoveModel.Goal.COLLISION_MODE_PAUSE,
            interpolation_mode=MoveModel.Goal.INTERPOLATION_MODE_CUBIC_SPLINE),
            self.process_feedback)
        rclpy.spin_until_future_complete(node, future_response)
        self.move_model_goal_handle = future_response.result()  # type: ClientGoalHandle
        if self.move_model_goal_handle.accepted:
            self.move_model_goal_handle.get_result_async().add_done_callback(self.process_result)

    def stop(self, node: Node):
        self.move_model_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, self.move_model_goal_handle.get_result_async())

    def process_feedback(self, feedback_msg: MoveModel.Impl.FeedbackMessage):
        if feedback_msg.feedback.iteration != self.laps:
            self.laps = feedback_msg.feedback.iteration
            if self.laps == 1:
                self.logger.info(f'forklift "{self.id}" has completed its first lap')
            else:
                self.logger.info(f'forklift "{self.id}" has completed {self.laps} laps')

    def process_result(self, future_result: Future):
        result = future_result.result().result  # type: MoveModel.Result
        status_name = get_name_of_class_constant(MoveModel.Result, 'STATUS_', result.status,
                                                 result.status)
        self.logger.info(f'forklift finished moving with status {status_name}')


class FactoryControlNode(Node):
    def __init__(self):
        super().__init__('factory_control')
        self.create_model_client = self.create_client(CreateModel, 'stage_ros2/create_model')
        self.remove_models_client = self.create_client(RemoveModels, 'stage_ros2/remove_models')
        self.move_model_client = ActionClient(self, MoveModel, 'stage_ros2/move_model')
        self.create_model_client.wait_for_service()
        self.remove_models_client.wait_for_service()
        self.move_model_client.wait_for_server()
        self.conveyed_items = [
            ConveyedItem(self, i, self.create_model_client, self.move_model_client)
            for i in range(15)]
        self.doors = [SlidingDoor(self, 'door_a', Pose(position=Point(x=12.75, y=12.5)),
                                  Pose(position=Point(x=12.75, y=10.5)), 1, 6, 12,
                                  self.move_model_client),
                      SlidingDoor(self, 'door_b', Pose(position=Point(x=12.75, y=2.5)),
                                  Pose(position=Point(x=12.75, y=4.5)), 5, 10, 12,
                                  self.move_model_client)]
        self.forklift = Forklift(self, 'forklift', self.move_model_client)

    def destroy_node(self):
        for conveyed_item in self.conveyed_items:
            conveyed_item.stop(self)
        rclpy.spin_until_future_complete(self, self.remove_models_client.call_async(
            RemoveModels.Request(ids=[conveyed_item.id for conveyed_item in self.conveyed_items])))
        for door in self.doors:
            door.stop(self)
        self.forklift.stop(self)
        super().destroy_node()


def main():
    rclpy.init()
    node = FactoryControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
