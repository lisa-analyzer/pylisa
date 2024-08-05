#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from awr_ros_interfaces.msg import (
    DirectorScript,
)
from awr_ros_interfaces.srv import (
    UpdateScript,
)
from awr_learning import (
    ConversationDirector,
)


class ConversationDirectorNode(Node):
    """Conversation psychologist to evaluate the personality of the user."""

    def __init__(self):
        super().__init__("ConversationDirectorNode")
        self.evaluate_personality_service = self.create_service(
            UpdateScript,
            "director/update_script",
            self.update_script_callback,
        )
        self.declare_parameter("temperature", 0.0)
        temperature = self.get_parameter("temperature").value
        self.conversation_director = ConversationDirector(temperature=temperature)

        self.get_logger().info("Conversation director node started")

    def update_script_callback(self, request, response):
        self.get_logger().info("Asked to update script")
        script = self.conversation_director.get_script(
            actor_description=request.actor_description,
            person_name=request.person_name,
            conversation_summary=request.conversation_summary,
            learning_stage_goals=request.learning_stage_goals,
            long_term_goals=request.long_term_goals,
            hypotheses=request.hypotheses,
            anticipation=request.anticipation,
            recommendations=request.recommendations,
        )
        response.script = self.parse_script(script)
        
        # Append here the result of the company decision.
        # this way we have the brain reporting how to success in the task that we need.
        # Lets continue with the show.
        
        # So go to to the AGI folder, place it here as it was the reasoning module.
        # Just finish this first, then copy aware and past it here.
        
        
        response.success = True
        return response
    
    # LINK HERE WITH AWARE REASONING! or link at INTEGRATOR
    def update_reasoning_feedback():
        return None # ADDED FOR TESTING

    def parse_script(self, script):
        director_script_msg = DirectorScript()
        director_script = message_to_ordereddict(director_script_msg)
        for key, value in script.items():
            if key in director_script:
                director_script[key] = value
        self.get_logger().info(f"Director script: {director_script}")
        set_message_fields(director_script_msg, director_script)
        return director_script_msg


def main(args=None):
    rclpy.init(args=args)
    conversation_director_node = ConversationDirectorNode()
    rclpy.spin(conversation_director_node)
    conversation_director_node.destroy_node()
    rclpy.shutdown()
main() # ADDED FOR TESTING