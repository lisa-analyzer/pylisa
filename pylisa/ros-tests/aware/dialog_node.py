import queue
import rclpy
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String

from aware_action import ConversationActor
from aware_memory import DialogMemory
from awr_ros_interfaces.action import Dialog
from awr_ros_interfaces.msg import (
    BehavioralContext,
    Context,
    ConversationTurn,
    DialogMode,
)
from awr_ros_interfaces.srv import (
    ClearChatHistory,
    RetrieveChatHistory,
    RetrieveContext,
    RetrieveHypotheses,
    RetrieveLearningGoals,
    RetrieveRelevantInformation,
    RetrieveRelationshipData,
)
import time


class DialogNode(Node):
    """Dialog node, hri entry point to our cognitive agent."""

    def __init__(self):
        super().__init__("dialog_node")
        self.declare_parameter("temperature", 0.0)
        self.group = ReentrantCallbackGroup()
        self._dialog_action_server = ActionServer(
            self,
            Dialog,
            "dialog/get_response",
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.stop_dialog_callback,
            callback_group=self.group,
        )
        self.listen_subscription = self.create_subscription(
            String, "/perception/listen", self.listener_callback, 10
        )
        self.web_user_input_subscription = self.create_subscription(
            String, "/web/user_input", self.manual_user_input_callback, 10
        )
        self.clear_chat_history_server = self.create_service(
            ClearChatHistory,
            "dialog/clear_chat_history",
            self.clear_chat_history_callback,
            callback_group=self.group,
        )
        self.retrieve_chat_history_server = self.create_service(
            RetrieveChatHistory,
            "dialog/retrieve_chat_history",
            self.retrieve_chat_history_callback,
            callback_group=self.group,
        )
        # Short term memory
        self.conversation_turns_publisher = self.create_publisher(
            ConversationTurn, "dialog/conversation_turn", 10
        )
        self.context_subscription = self.create_subscription(
            Context, "short_term_memory/context", self.update_context_callback, 10
        )
        # Long term memory
        self.retrieve_relevant_information_client = self.create_client(
            RetrieveRelevantInformation,
            "long_term_memory/retrieve_relevant_information",
            callback_group=self.group,
        )
        self.retrieve_relationship_data_client = self.create_client(
            RetrieveRelationshipData,
            "long_term_memory/retrieve_relationship_data",
            callback_group=self.group,
        )
        self.retrieve_hypotheses_client = self.create_client(
            RetrieveHypotheses,
            "long_term_memory/retrieve_hypotheses",
            callback_group=self.group,
        )
        # Learning
        self.retrieve_learning_goals_client = self.create_client(
            RetrieveLearningGoals,
            "learning/retrieve_learning_goals",
            callback_group=self.group,
        )
        # Dialog agent v.2.0, for now just use the actor chain.
        # self.dialog_agent = DialogAgent()
        self.conversation_actor = ConversationActor(
            temperature=self.get_parameter("temperature").value
        )
        # Conversation context
        self.behavioral_context = BehavioralContext()
        self._conversation_goal_handle = None
        # User input queue
        self.user_input_queue = queue.Queue()
        # Dialog agent job
        self.dialog_thread = threading.Thread(target=self.dialog_job).start()
        # Retrieve context on start
        self.retrieve_context_client = self.create_client(
            RetrieveContext,
            "short_term_memory/retrieve_context",
        )
        if self.retrieve_context_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().info("Sending request to retrieve self description")
            request = RetrieveContext.Request()
            future = self.retrieve_context_client.call_async(request)
            while not future.done():
                rclpy.spin_once(self)  # Force to get self description before starting
                time.sleep(0.1)
            response = future.result()
            if response.success:
                self_description = response.context.self_description
                self.dialog_working_memory = DialogMemory(
                    name=self_description.name,
                    self_description=self_description.self_description,
                    long_term_goals=self_description.long_term_goals,
                )
                self.get_logger().info("Context retrieved")
            else:
                self.get_logger().error("Critical error! Failed to retrieve context.")
        else:
            self.get_logger().error(
                "Long term memory is not running! Can't execute dialog. Critical error!"
            )
        self.get_logger().info("Dialog node started")

    def update_conversation_context(self, context_msg):
        person_name = self.dialog_working_memory.retrieve_person_name()
        # Get latest hypotheses, TODO: Make this situational based on latest turn.
        hypotheses_future = self.retrieve_hypotheses_client.call_async(
            RetrieveHypotheses.Request(person_name=person_name)
        )
        while not hypotheses_future.done():
            time.sleep(0.1)
        if hypotheses_future.result().success:
            self.get_logger().info("Got hypotheses from long term memory")
            self.dialog_working_memory.update_hypotheses(
                hypotheses_future.result().hypotheses
            )
        self.behavioral_context = context_msg.behavioral_context
        learning_goals_response = self.retrieve_learning_goals(person_name)
        if learning_goals_response.success:
            self.dialog_working_memory.update_learning_goals(
                stage_goals=learning_goals_response.learning_goals,
                examples=learning_goals_response.examples,
            )
        relationship_data_future = self.retrieve_relationship_data_client.call_async(
            RetrieveRelationshipData.Request(person_name=person_name)
        )
        while not relationship_data_future.done():
            time.sleep(0.1)
        if relationship_data_future.result().success:
            relationship_data = relationship_data_future.result().data
            person_data = message_to_ordereddict(relationship_data.personality_data)
            self.dialog_working_memory.update_person_data(person_data)
            self.dialog_working_memory.update_expected_role(
                relationship_data.user_interaction_data.expected_role
            )

    def clear_chat_history_callback(self, request, response):
        self.dialog_working_memory.clear()
        response.success = True
        return response

    def handle_accepted_callback(self, goal_handle):
        """Defer goal handle execution saving it and sending the feedback after \
        processing new inputs"""
        self.get_logger().info("Goal accepted")
        self._conversation_goal_handle = goal_handle
        self.initialize_conversation(person_name=goal_handle.request.person_name)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Goal executing")
        result = Dialog.Result(
            person_name=self.dialog_working_memory.retrieve_person_name(),
            conversation=self.dialog_working_memory.retrieve_chat_history(),
        )
        goal_handle.succeed()
        return result

    def stop_dialog_callback(self, cancel_request):
        if self._conversation_goal_handle:
            self._conversation_goal_handle.execute()
            self._conversation_goal_handle = None
        return CancelResponse.REJECT

    def listener_callback(self, msg):
        if (
            self._conversation_goal_handle
            and self._conversation_goal_handle.request.dialog_mode.mode
            == DialogMode.MODE_AUTOMATIC
        ):
            self.get_logger().info(f"Received: {msg.data}")
            self.user_input_queue.put(msg.data)

    def manual_user_input_callback(self, msg):
        if (
            self._conversation_goal_handle
            and self._conversation_goal_handle.request.dialog_mode.mode
            == DialogMode.MODE_MANUAL
        ):
            self.get_logger().info(f"Received manual input: {msg.data}")
            self.user_input_queue.put(msg.data)

    def update_context_callback(self, context_msg):
        self.update_conversation_context(context_msg)

    def initialize_conversation(self, person_name):
        self.get_logger().info("Initializing conversation")
        future = self.retrieve_context_client.call_async(RetrieveContext.Request())
        while not future.done():
            time.sleep(0.1)
        context_response = future.result()
        context = context_response.context
        self.update_conversation_context(context)
        self.dialog_working_memory.initialize_conversation(person_name=person_name)
        self.get_logger().info(f"Conversation initialized with person: {person_name}")

    def retrieve_learning_goals(self, person_name):
        self.get_logger().info("Retrieving learning goals")
        future = self.retrieve_learning_goals_client.call_async(
            RetrieveLearningGoals.Request(person_name=person_name)
        )
        while not future.done():
            time.sleep(0.1)
        learning_goals_response = future.result()
        return learning_goals_response

    def retrieve_chat_history_callback(self, request, response):
        if self.dialog_working_memory:
            response.chat_history = self.dialog_working_memory.retrieve_chat_history()
            response.person_name = self.dialog_working_memory.retrieve_person_name()
            response.success = True
        else:
            response.success = False
        return response

    def get_response(self, user_msg):
        try:
            person_name = self.dialog_working_memory.retrieve_person_name()
            personality_data = self.dialog_working_memory.retrieve_person_data()
            user_information = ""
            for key, value in personality_data.items():
                user_information += f"{key}: {value}\n"
            response = self.conversation_actor.answer_to_user(
                user_name=person_name,
                user_information=user_information,
                short_term_goals=self.dialog_working_memory.retrieve_short_term_goals(),
                name=self.dialog_working_memory.retrieve_name(),
                self_description=self.dialog_working_memory.retrieve_self_description(),
                emotions=self.behavioral_context.emotions,
                thoughts=self.behavioral_context.thoughts,
                immediate_goals=self.behavioral_context.goals,
                recommendations=self.behavioral_context.recommendations,
                hypotheses=self.dialog_working_memory.retrieve_hypotheses(),
                example=self.dialog_working_memory.retrieve_examples(),
                expected_role=self.dialog_working_memory.retrieve_expected_role(),
                chat_history=self.dialog_working_memory.retrieve_chat_history(),
                user_msg=user_msg,
            )
            self.dialog_working_memory.store_conversation_turn(user_msg, response)
            conversation_turn = ConversationTurn()
            conversation_turn.person_name = person_name
            conversation_turn.human_msg = user_msg
            conversation_turn.ai_msg = response
            self.conversation_turns_publisher.publish(conversation_turn)
            return response
        except Exception as e:
            error_msg = f"Failed to get response with error {e}"
            self.get_logger().error(error_msg)
            return error_msg

    def get_relevant_information(self, query: str) -> str:
        self.get_logger().info(f"Requesting memory relevant info for query: {query}")
        request = RetrieveRelevantInformation.Request()
        request.person_name = self.person_name
        request.query = query
        response = self.retrieve_relevant_information_client.call(request)
        if response.success:
            self.get_logger().info(f"Relevant info: {response.relevant_information}")
            return response.relevant_information
        else:
            self.get_logger().info("Communication with memory failed.")
            return "Memory did not return any relevant information."

    def dialog_job(self):
        while rclpy.ok():
            if self._conversation_goal_handle and not self.user_input_queue.empty():
                user_input = self.user_input_queue.get(block=False)
                self.get_logger().info(f"Processing message: {user_input}")
                response = self.get_response(user_msg=user_input)
                self._conversation_goal_handle.publish_feedback(
                    Dialog.Feedback(response=response)
                )
                self.get_logger().info(f"Response: {response}")
            else:
                time.sleep(0.1)  # Slow down a bit


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=8)
    dialog_node = DialogNode()
    executor.add_node(dialog_node)
    executor.spin()
    executor.shutdown()
    dialog_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
