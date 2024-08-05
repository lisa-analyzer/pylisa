import rclpy
from rclpy.node import Node
from aware_libs.aware_libs.action import Talk
from rclpy.action import ActionServer, GoalResponse, CancelResponse, GoalStatus
from playsound import playsound

from awr_action import TextToSpeech


class TalkNode(Node):
    def __init__(self):
        super().__init__("awr_talk_node")
        self.tts = TextToSpeech(
            "NEMO", "/aware/dialogue/models"
        )  # TODO: Retrieve from ros config.
        self.get_logger().info("Loading model")
        self.tts.load()
        self._action_server = ActionServer(
            self,
            Talk,
            "talk",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self._goal_handle = None
        self._goal_accepted = False
        self.get_logger().info("Talk node started")

    def goal_callback(self, goal_handle):
        if self._goal_accepted:
            self.get_logger().warn("Another goal is currently being executed")
            return GoalResponse.REJECT
        self._goal_accepted = True
        self._goal_handle = goal_handle
        self.get_logger().info(f"Received goal: {goal_handle.request.text}")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Talking: {goal_handle.request.text}")
        audio = self.tts.get_audio(goal_handle.request.text)
        audio_path = self.tts.save_audio(audio)
        playsound(audio_path, block=True)
        return Talk.Result()

    def cancel_callback(self, goal_handle):
        self.get_logger().info(
            f"Goal {goal_handle} has been canceled"
        )  # TODO: Verify if we should cancel during mic execution.
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    talk_node = TalkNode()
    rclpy.spin(talk_node)
    talk_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
