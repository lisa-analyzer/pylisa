#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from awr_ros_interfaces.msg import (
    PersonalityTraits,
    PsychologistFeedback,
)

from awr_ros_interfaces.srv import (
    EvaluatePersonality,
)

from awr_learning import (
    ConversationPsychologist,
)


class ConversationPsychologistNode(Node):
    """Conversation psychologist to evaluate the personality of the user."""

    def __init__(self):
        super().__init__("ConversationPsychologistNode")
        self.evaluate_personality_service = self.create_service(
            EvaluatePersonality,
            "psychologist/evaluate_personality",
            self.evaluate_personality_callback,
        )
        self.declare_parameter("temperature", 0.0)
        temperature = self.get_parameter("temperature").value
        self.conversation_pyschologist = ConversationPsychologist(
            temperature=temperature
        )

        self.get_logger().info("Conversation psychologist node started")

    def evaluate_personality_callback(self, request, response):
        personality_data = message_to_ordereddict(request.personality_data)
        user_information = ""
        for key, value in personality_data.items():
            user_information += f"{key}: {value}\n"

        personality_evaluation = self.conversation_pyschologist.evaluate_personality(
            conversation=request.conversation,
            user_information=user_information,
            person_name=request.person_name,
        )
        personality_traits_msg, psychologist_feedback_msg = self.parse_evaluation(
            personality_evaluation
        )

        response.personality_traits = personality_traits_msg
        response.feedback = psychologist_feedback_msg
        response.success = True
        return response

    def parse_evaluation(self, evaluation):
        # Personality traits metrics message
        personality_traits_msg = PersonalityTraits()
        personality_traits = message_to_ordereddict(personality_traits_msg)
        # Psychologist feedback message
        psychologist_feedback_msg = PsychologistFeedback()
        psychologist_feedback = message_to_ordereddict(psychologist_feedback_msg)
        for key, value in evaluation.items():
            if key in personality_traits.keys():
                personality_traits[key] = value
            elif key in psychologist_feedback.keys():
                psychologist_feedback[key] = value
            else:
                self.get_logger().error(f"Unknown key {key} in evaluation")
        set_message_fields(personality_traits_msg, personality_traits)
        self.get_logger().info("Personality traits: " + str(personality_traits))
        set_message_fields(psychologist_feedback_msg, psychologist_feedback)
        self.get_logger().info("Psychologist feedback: " + str(psychologist_feedback))
        return personality_traits_msg, psychologist_feedback_msg


def main(args=None):
    rclpy.init(args=args)
    conversation_psychologist_node = ConversationPsychologistNode()
    rclpy.spin(conversation_psychologist_node)
    conversation_psychologist_node.destroy_node()
    rclpy.shutdown()

main() # ADDED FOR TESTING