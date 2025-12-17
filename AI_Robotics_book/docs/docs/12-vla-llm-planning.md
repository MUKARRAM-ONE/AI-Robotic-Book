---
id: 12-vla-llm-planning
title: "Chapter 12: Cognitive Planning with LLMs"
sidebar_label: "12. Cognitive Planning with LLMs"
---

## Chapter 12: Cognitive Planning with LLMs

**Objective**: Use a Large Language Model to translate a high-level goal into a sequence of robot actions.

![VLA Sequence Diagram](/img/module4-vla-sequence.svg)
![VLA Architecture](/img/VLA.jpg)
![VLA Flow](/img/VLA Arch.webp)
![Open VLA](/img/OPEN_VLA.jpg)

### 12.1 Introduction to LLMs in Robotics

Large Language Models (LLMs) have emerged as powerful tools for understanding and generating human language. Their ability to comprehend complex instructions, infer intent, and generate coherent text makes them incredibly valuable for extending the cognitive capabilities of robots. In the context of robotics, LLMs can act as a "high-level brain," translating abstract human commands into concrete, executable sequences of robot actions.

Key benefits of LLMs in robotics:
-   **Natural Language Interface**: Enables humans to interact with robots using everyday language.
-   **Cognitive Planning**: Breaking down complex tasks into simpler sub-tasks.
-   **Context Understanding**: Incorporating environmental and task context to refine plans.
-   **Error Recovery**: Suggesting alternative actions or seeking clarification during execution failures.

### 12.2 Prompt Engineering for Robot Action Sequences

The quality of an LLM's output is highly dependent on the **prompt** you provide. **Prompt engineering** is the art and science of crafting effective prompts to guide the LLM towards generating desired outputs. For robotics, this means designing prompts that elicit structured, executable action sequences.

#### Key Principles for Robotic Prompts
-   **Clear Instructions**: Explicitly state the desired output format (e.g., "Respond only with a JSON array of ROS 2 actions.").
-   **Role-Playing**: Assign the LLM a role (e.g., "You are a robot task planner.").
-   **Few-Shot Examples**: Provide examples of input commands and their corresponding valid action sequences.
-   **Constraints**: Specify rules for valid actions, available tools, and environmental limitations.
-   **Safety Guidelines**: Include instructions to prioritize safety and avoid dangerous actions.

#### Example Prompt Structure
```
"You are a robot task planner. Your goal is to convert natural language commands into a sequence of ROS 2 actions.
Available ROS 2 Actions:
- navigate_to_pose(x, y, yaw_degrees)
- pick_up_object(object_name)
- say_phrase(phrase)

Respond ONLY with a JSON array where each object is an action.

Example:
User: Go to the kitchen and pick up the apple.
Response:
[
  {"action": "navigate_to_pose", "params": {"x": 5.0, "y": 2.0, "yaw_degrees": 90.0}},
  {"action": "pick_up_object", "params": {"object_name": "apple"}}
]

User: {natural_language_command_from_whisper}
Response:
```

### 12.3 Integrating with an LLM API

Similar to Whisper, you can integrate with an LLM API (e.g., OpenAI's GPT models) via their Python client library.

We'll create a ROS 2 node that:
1.  Subscribes to the transcribed text topic (e.g., `/voice_command/text`).
2.  Constructs a prompt based on the received command and robot context.
3.  Sends the prompt to the LLM API.
4.  Parses the LLM's JSON response into a sequence of ROS 2 actions.
5.  Publishes these actions to a new ROS 2 topic or directly calls action clients.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json

# This conceptual node subscribes to transcribed voice commands and uses an LLM
# to generate a sequence of ROS 2 actions.

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.subscription = self.create_subscription(
            String,
            '/voice_command/text',
            self.command_callback,
            10
        )
        self.action_sequence_publisher = self.create_publisher(String, '/robot_action_sequence', 10)
        self.get_logger().info('Cognitive Planner node started.')

        openai.api_key = os.getenv("OPENAI_API_KEY", "YOUR_OPENAI_API_KEY") # Ensure API key is set

        self.robot_context = {
            "current_location": "living_room",
            "available_objects": ["apple", "cup", "book"],
            "known_locations": {"kitchen": (5.0, 2.0, 90.0), "bedroom": (-3.0, 4.0, 0.0)}
        }

    def command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f"Received voice command: '{command_text}'")
        
        # Construct prompt for the LLM
        prompt = self.create_llm_prompt(command_text)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", # or gpt-4
                messages=[
                    {"role": "system", "content": "You are a robot task planner. Convert commands into ROS 2 actions."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=200,
                temperature=0.0 # Make it deterministic
            )
            llm_response_content = response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM Response: {llm_response_content}")

            action_sequence = json.loads(llm_response_content)
            
            # Publish the action sequence for execution
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_sequence_publisher.publish(action_msg)
            self.get_logger().info("Published ROS 2 action sequence.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"LLM response not valid JSON: {e}")
            self.get_logger().error(f"Raw LLM response: {llm_response_content}")
        except Exception as e:
            self.get_logger().error(f"Error calling LLM API: {e}")

    def create_llm_prompt(self, command_text):
        # This is where prompt engineering happens
        # It should include current robot context and available actions
        prompt = f"""You are a robot task planner. Your goal is to convert natural language commands into a sequence of ROS 2 actions.
Available ROS 2 Actions:
- navigate_to_pose(x, y, yaw_degrees): Navigate to a 2D pose (x,y) with a final orientation (yaw_degrees).
- pick_up_object(object_name): Pick up a specified object.
- find_object(object_name): Find a specified object using perception.
- say_phrase(phrase): Make the robot speak a phrase.

Current robot location: {self.robot_context['current_location']}
Known locations: {json.dumps(self.robot_context['known_locations'])}
Available objects: {json.dumps(self.robot_context['available_objects'])}

Respond ONLY with a JSON array where each object is an action. Do not include any other text.
If a location is mentioned, use its coordinates from 'known_locations'.
If you cannot fulfill the command with available actions, return an empty array.

User: {command_text}
Response:
"""
        return prompt

def main(args=None):
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlannerNode()
    rclpy.spin(cognitive_planner_node)
    cognitive_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 12.4 Translating LLM Output to ROS 2 Actions

The `action_sequence` generated by the LLM is a structured representation of the robot's plan. A separate **Action Executor** node will be responsible for:
1.  **Parsing the JSON**: Decoding the LLM's action sequence.
2.  **Mapping to ROS 2 Primitives**: Calling the appropriate ROS 2 action clients, service clients, or publishing to topics based on the parsed action definitions.
3.  **Monitoring Execution**: Tracking the status of each ROS 2 action (e.g., navigation goal status, service call success).
4.  **Error Handling**: If an action fails, the executor might trigger a recovery behavior or report back to the cognitive planner (potentially leading to a re-plan).

This layered approach, from voice command to transcribed text, to an LLM-generated action sequence, and finally to ROS 2 execution, allows for highly flexible and intelligent robot control.
