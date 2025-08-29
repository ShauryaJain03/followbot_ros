#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM
import threading


class LogInference(Node):
    def __init__(self):
        super().__init__('log_inference')
        self.model_loaded = False
        self.model = None
        self.tokenizer = None

        self.subscription = self.create_subscription(
            String,
            '/bot/log',
            self.json_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/bot/explanation', 10)

        self.model_thread = threading.Thread(target=self.load_model, daemon=True)
        self.model_thread.start()

    def load_model(self):
        try:
            self.get_logger().info('Loading TinyLlama model...')
            model_name = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"

            self.tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
            self.model = AutoModelForCausalLM.from_pretrained(
                model_name,
                torch_dtype=torch.float16,
                device_map="auto",
                trust_remote_code=True
            )

            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token

            self.model_loaded = True
            self.get_logger().info('Model loaded successfully!')

        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')

    def json_callback(self, msg):
        if not self.model_loaded:
            self.get_logger().warn('Model still loading, please wait...')
            return

        try:
            robot_data = json.loads(msg.data)
            self.get_logger().info(f'Received: {robot_data}')
            explanation = self.convert_to_explanation(robot_data)

            explanation_msg = String()
            explanation_msg.data = explanation
            self.publisher.publish(explanation_msg)

            self.get_logger().info(f'Explanation: "{explanation}"')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON received')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')

    def convert_to_explanation(self, robot_data):
        try:
            mode = robot_data.get('mode', 'unknown')
            battery = robot_data.get('battery_pct', 'unknown')
            pose = robot_data.get('pose', {})
            active_path = robot_data.get('active_path', [])
            task = robot_data.get('current_task', 'unknown task')
            status = robot_data.get('status', 'unknown status')
            timestamp = robot_data.get('timestamp', 'unknown time')

            prompt = f"""<|system|>
            You are a helpful robot explaining your status to humans in simple language.
            <|user|>
            Mode: {mode}
            Battery: {battery}%
            Current Task: {task}
            Status: {status}
            Active Path: {active_path}
            Pose: {pose}
            Time: {timestamp}
            Explain this in 1-2 simple sentences as if talking to your human operator.
            <|assistant|>"""

            inputs = self.tokenizer(prompt, return_tensors="pt", padding=True)

            with torch.no_grad():
                outputs = self.model.generate(
                    inputs.input_ids,
                    attention_mask=inputs.attention_mask,
                    max_new_tokens=50,
                    temperature=0.3,
                    do_sample=True,
                    pad_token_id=self.tokenizer.eos_token_id,
                    eos_token_id=self.tokenizer.eos_token_id
                )

            full_response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
            explanation = full_response.split("<|assistant|>")[-1].strip()

            sentences = explanation.split('.')[:2]
            clean_explanation = '. '.join([s.strip() for s in sentences if s.strip()])

            if clean_explanation and not clean_explanation.endswith('.'):
                clean_explanation += '.'

            return clean_explanation if clean_explanation else f"My mode is {mode}, task is {task}, and battery is {battery}%."

        except Exception as e:
            self.get_logger().error(f'Error generating explanation: {str(e)}')
            return f"My current task is {robot_data.get('current_task', 'unknown')} with status {robot_data.get('status', 'unknown')}."


def main(args=None):
    rclpy.init(args=args)
    node = LogInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
