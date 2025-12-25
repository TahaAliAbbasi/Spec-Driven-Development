---
title: کیپسٹون پروجیکٹ نمونہ حل ٹیمپلیٹ
sidebar_position: 4
---

# کیپسٹون پروجیکٹ نمونہ حل ٹیمپلیٹ

یہ دستاویز کیپسٹون پروجیکٹ کے لیے ایک بنیادی ٹیمپلیٹ اور مثالی آرکیٹیکچر فراہم کرتی ہے۔ یہ ظاہر کرتا ہے کہ آپ ROS 2 ورک اسپیس کو کیسے ساخت بندی کریں اور کلیدی جز کو کیسے یکجا کریں۔ یہ *ایک ممکنہ نقطہ نظر* ہے اور طلباء کو اس کو اپنایا جانے اور اس کو وسعت دینے کے لیے حوصلہ دیا جاتا ہے۔

## پروجیکٹ ساخت

آپ کا ROS 2 ورک اسپیس ایک معیاری ساخت کا پیرو کرنا چاہیے:

```
capstone_ws/
├── src/
│   ├── capstone_bringup/         # لانچ فائلیں پورے سسٹم کو شروع کرنے کے لیے
│   │   ├── launch/
│   │   │   └── capstone_demo.launch.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── nl_command_interpreter/   # قدرتی زبان کی پروسیسنگ کے لیے نوڈ
│   │   ├── src/
│   │   │   └── nl_interpreter_node.py
│   │   ├── config/
│   │   │   └── openai_params.yaml
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── task_planner/             # LLM کے ساتھ ذہنی منصوبہ بندی کے لیے نوڈ
│   │   ├── src/
│   │   │   └── task_planner_node.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── perception_node/          # چیز کا پتہ لگانے کے لیے نوڈ
│   │   ├── src/
│   │   │   └── simple_detector.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   └── manipulation_controller/  # بنیادی مینیپولیشن کے لیے نوڈ
│       ├── src/
│       │   └── simple_manipulator.py
│       └── CMakeLists.txt
│       └── package.xml
```

## مثالی جز

### 1. لانچ فائل (`capstone_bringup/launch/capstone_demo.launch.py`)

یہ فائل ڈیمو کے لیے ضروری تمام نوڈز کو شروع کرتی ہے۔

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # کنفیگ فائل کا راستہ (اگر ضرورت ہو)
    # config = os.path.join(get_package_share_directory('nl_command_interpreter'), 'config', 'openai_params.yaml')

    return LaunchDescription([
        # قدرتی زبان کے حکم کی تشریح کے لیے نوڈ
        Node(
            package='nl_command_interpreter',
            executable='nl_interpreter_node',
            name='nl_interpreter',
            # parameters=[config] # اگر کنفیگ فائل استعمال کر رہے ہیں تو ان کو غیر تبصرہ کریں
        ),

        # LLM کا استعمال کرتے ہوئے کام کی منصوبہ بندی کے لیے نوڈ
        Node(
            package='task_planner',
            executable='task_planner_node',
            name='task_planner',
        ),

        # ادراک (چیز کا پتہ لگانا) کے لیے نوڈ
        Node(
            package='perception_node',
            executable='simple_detector',
            name='simple_detector',
            # مثالی ری میپنگ: ROS2 ٹاپک سے شبیہ سازی سے آپ کے نوڈ کی سبسکرپشن تک
            remappings=[('/camera/image_raw', '/your_sim_camera_topic')]
        ),

        # مینیپولیشن کے لیے نوڈ
        Node(
            package='manipulation_controller',
            executable='simple_manipulator',
            name='simple_manipulator',
        ),

        # مثال: Nav2 شروع کریں (یہ تصور کرتے ہوئے کہ اس کو آپ کے روبوٹ/نقشہ کے لیے کنفیگر کر دیا گیا ہے)
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager',
        #     parameters=[{'use_sim_time': True}] # اگر شبیہ وقت استعمال کر رہے ہیں تو یہ اہم ہے
        # ),
        # دیگر Nav2 نوڈز شامل کریں (amcl، planner_server، controller_server، وغیرہ) آپ کے Nav2 سیٹ اپ کی بنیاد پر جیسے کہ ضرورت ہو۔
    ])

```

### 2. قدرتی زبان کا تشریح کار نوڈ (`nl_command_interpreter/src/nl_interpreter_node.py`)

یہ ایک مفہومی مثال ہے۔ یہ ایک متن کمانڈ حاصل کر سکتا ہے اور اس کو ایک منصوبہ بندی نوڈ میں شائع کر سکتا ہے۔

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai # 'pip install openai' کی ضرورت ہے

class NLInterpreterNode(Node):
    def __init__(self):
        super().__init__('nl_interpreter_node')
        # منصوبہ بندی کے نوڈ میں پروسیسڈ کمانڈ بھیجنے کے لیے پبلشر
        self.planner_publisher = self.create_publisher(String, 'planning/command', 10)

        # اس مثال کے لیے، ہم ایک ٹائمر کے ذریعے کمانڈ حاصل کرنا تصور کریں گے
        # عمل میں، یہ دوسرے نوڈ یا ایک سروس کال کے ذریعے Whisper سے آ سکتا ہے۔
        self.i = 0
        self.timer = self.create_timer(5.0, self.timer_callback) # ہر 5 سیکنڈ میں شائع کریں

    def timer_callback(self):
        # مثال کمانڈ
        command_text = "Red cube کے پاس جائیں اور اسے اٹھا لیں۔"
        msg = String()
        msg.data = command_text
        self.planner_publisher.publish(msg)
        self.get_logger().info(f'کمانڈ شائع کر رہا ہے: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = NLInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 3. کام کا منصوبہ ساز نوڈ (`task_planner/src/task_planner_node.py`)

یہ نوڈ کمانڈ حاصل کرتا ہے، کام کو تقسیم کرنے کے لیے ایک LLM کا استعمال کرتا ہے، اور ایکسری ایکشنز شائع کر سکتا ہے۔

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai # 'pip install openai' کی ضرورت ہے

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')
        # NL تشریح کار سے کمانڈ حاصل کرنے کے لیے سبسکرائبر
        self.command_subscriber = self.create_subscription(
            String,
            'planning/command',
            self.command_callback,
            10
        )

        # ایکسری ایکشن (ایک حسب ضرورت میسج قسم ہو سکتی ہے) بھیجنے کے لیے پبلشر
        self.action_publisher = self.create_publisher(String, 'planning/action_sequence', 10)

    def command_callback(self, msg):
        self.get_logger().info(f'کمانڈ موصول ہوئی: "{msg.data}"')
        # کام کو ایکشنز میں تقسیم کرنے کے لیے LLM کو کال کریں
        actions = self.call_llm_for_planning(msg.data)
        if actions:
            action_msg = String()
            action_msg.data = json.dumps(actions) # ایکشنز کی فہرست کو سیریلائز کریں
            self.action_publisher.publish(action_msg)
            self.get_logger().info(f'ایکشن سیکوئنس شائع کیا گیا: {actions}')

    def call_llm_for_planning(self, command):
        # LLM کے لیے مثالی پروموٹ
        prompt = f"""
        آپ ایک روبوٹکس پروجیکٹ کے لیے ایک مددگار اسسٹنٹ ہیں۔ ہیومنوائڈ روبوٹ کے لیے قدرتی زبان کے حکم کو دیکھتے ہوئے، براہ کرم اس کو سادہ، قابلِ انجام ایکشنز کی ایک ترتیب میں تقسیم کریں۔ ایک JSON فہرست کے طور پر جواب دیں۔

        کمانڈ: "{command}"

        مثالی آؤٹ پٹ فارمیٹ:
        [
            {{"action": "navigate_to", "target": "kitchen"}},
            {{"action": "detect_object", "object_type": "red apple"}},
            {{"action": "move_arm_to_object", "object_position": "..."}},
            {{"action": "grasp_object"}}
        ]

        براہ کرم ایکشن سیکوئنس اب فراہم کریں۔
        """

        try:
            # API کال کریں (یقینی بنائیں کہ OPENAI_API_KEY ماحول میں سیٹ ہے)
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", # یا کوئی دوسرا مناسب ماڈل
                messages=[{"role": "user", "content": prompt}],
                max_tokens=200, # ضرورت کے مطابق ایڈجسٹ کریں
                temperature=0.1, # زیادہ تعین کاری کے لیے کم درجہ حرارت
            )
            # جواب کو تحلیل کریں
            content = response.choices[0].message['content'].strip()
            # LLM جواب سے JSON تحلیل کرنے کی کوشش کریں
            import re
            json_match = re.search(r'\[.*\]', content, re.DOTALL) # JSON ارے تلاش کریں
            if json_match:
                actions = json.loads(json_match.group(0))
                return actions
            else:
                self.get_logger().error(f'LLM جواب سے JSON تحلیل نہیں کی جا سکی: {content}')
                return None

        except Exception as e:
            self.get_logger().error(f'LLM کو کال کرنے میں خرابی: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 4. ادراک نوڈ (`perception_node/src/simple_detector.py`)

یہ نوڈ ایک کیمرہ ٹاپک کو سبسکرائب کرتا ہے اور بنیادی چیز کا پتہ لگانے کا کام کرتا ہے (یہاں ایک جگہ کی منطق استعمال کی جا رہی ہے)۔

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # پتہ چلنے والے نتائج کے لیے ایک حسب ضرورت میسج ہو سکتا ہے
import numpy as np
# مثال: اپنی ڈیٹیکشن لائبریری (مثلاً OpenCV، PyTorch، وغیرہ) درآمد کریں
# import cv2
# import torch

class SimpleDetectorNode(Node):
    def __init__(self):
        super().__init__('simple_detector')
        # کیمرہ امیج کے لیے سبسکرائبر
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # یقینی بنائیں کہ یہ آپ کے شبیہ کیمرہ ٹاپک سے مماثل ہو
            self.image_callback,
            10
        )
        # ڈیٹیکشن نتائج کے لیے پبلشر
        self.detection_publisher = self.create_publisher(String, 'perception/detections', 10)

    def image_callback(self, msg):
        # self.get_logger().info('ایک امیج موصول ہوئی')
        # --- اصل ڈیٹیکشن منطق کے لیے جگہ ---
        # sensor_msgs/Image کو OpenCV/Numpy فارمیٹ میں تبدیل کریں اگر ضرورت ہو
        # raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        # results = your_detection_model(raw_data) # مثلاً YOLO، Detectron2، وغیرہ کا استعمال کر کے

        # اس ٹیمپلیٹ کے لیے، ہم ایک ڈیٹیکشن نتیجہ کی شبیہ کریں گے
        detection_result = {
            "object_type": "red_cube", # مثالی ڈیٹیکٹڈ چیز
            "confidence": 0.95,
            "bbox": {"x_min": 100, "y_min": 150, "x_max": 200, "y_max": 250},
            "position_3d": {"x": 1.0, "y": 2.0, "z": 0.5} # مثالی 3D مقام (گہرائی/TF کی ضرورت ہے)
        }

        detection_msg = String()
        detection_msg.data = str(detection_result) # یا ایک حسب ضرورت میسج قسم استعمال کریں
        self.detection_publisher.publish(detection_msg)
        self.get_logger().info(f'ڈیٹیکشن شائع کی گئی: {detection_result}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 5. مینیپولیشن کنٹرولر نوڈ (`manipulation_controller/src/simple_manipulator.py`)

یہ نوڈ روبوٹ کے بازو یا گرپر کو حرکت دینے کے لیے کمانڈز حاصل کر سکتا ہے۔

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # مینیپولیشن گولز کے لیے ایک حسب ضرورت میسج ہو سکتا ہے
# مثال: MoveIt! یا جوائنٹ ٹریجکٹری کنٹرول کے لیے میسج درآمد کریں
# from moveit_msgs.msg import MoveGroupGoal
# from trajectory_msgs.msg import JointTrajectory

class SimpleManipulatorNode(Node):
    def __init__(self):
        super().__init__('simple_manipulator')
        # مینیپولیشن کمانڈز کے لیے سبسکرائبر (کام کے منصوبہ ساز یا ادراک سے آ سکتا ہے)
        self.manipulation_subscriber = self.create_subscription(
            String,
            'manipulation/command', # مثالی ٹاپک
            self.manipulation_callback,
            10
        )
        # مثال: جوائنٹ ٹریجکٹری کمانڈز کے لیے پبلشر
        # self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    def manipulation_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'مینیپولیشن کمانڈ موصول ہوئی: {command}')
        # --- اصل مینیپولیشن منطق کے لیے جگہ ---
        # کمانڈ تحلیل کریں (مثلاً "grasp_object_at 1.0 2.0 0.5")
        # MoveIt! Python انٹرفیس استعمال کریں یا JointTrajectory میسج شائع کریں
        # مثالی نکوڈ:
        # if "grasp" in command:
        #     self.execute_grasp()
        # elif "move_to" in command:
        #     target_pos = parse_position(command)
        #     self.move_arm_to(target_pos)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleManipulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## تعمیر اور چلانا

1.  **ROS 2 کو سورس کریں**: یقینی بنائیں کہ آپ کا ROS 2 ماحول سورس کیا گیا ہے۔
    ```bash
source /opt/ros/humble/setup.bash # یا آپ کا ROS 2 ڈسٹرو
    ```
2.  **ورک اسپیس کی تعمیر**:
    ```bash
cd /path/to/capstone_ws
colcon build --packages-select capstone_bringup nl_command_interpreter task_planner perception_node manipulation_controller
    ```
3.  **ورک اسپیس کو سورس کریں**:
    ```bash
source install/setup.bash
    ```
4.  **لانچ فائل چلائیں**:
    ```bash
ros2 launch capstone_bringup capstone_demo.launch.py
    ```
    یقینی بنائیں کہ آپ کا شبیہ سازی ماحول (Gazebo/Isaac Sim/Unity) چل رہا ہے اور ROS 2 کے ساتھ انٹرفیس کرنے کے لیے کنفیگر کیا گیا ہے قبل اس نوڈز کو لانچ کرنے کے۔

یہ ٹیمپلیٹ ایک بنیادی سکلیٹن فراہم کرتا ہے۔ آپ کو "جگہ" حصوں کو اصل منطق کے ساتھ بھرنا ہوگا، اپنے مخصوص روبوٹ اور شبیہ سازی ماحول کو کنفیگر کرنا ہوگا، اور ممکنہ طور پر ROS 2 کے حسب ضرورت میسج قسمیں تخلیق کرنا ہوگی نوڈز کے درمیان پیچیدہ ڈیٹا ایکسچینج کے لیے۔