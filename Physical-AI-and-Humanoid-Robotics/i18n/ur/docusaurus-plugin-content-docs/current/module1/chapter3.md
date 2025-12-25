---
title: "باب 3: rclpy کی یک جہتی - سیکھنے کے نتائج"
sidebar_position: 3
---

# باب 3: `rclpy` کی یک جہتی - سیکھنے کے نتائج

یہ باب `rclpy` پر مرکوز ہے، ROS 2 کے لیے پائی تھون کلائنٹ لائبریری، اور اس کی AI ایجنٹس تیار کرنے اور URDF ماڈلز کو یکجا کرنے میں اطلاق کے بارے میں۔ تکمیل کے بعد، طلباء کے قابل ہو جائیں گے:

*   **`rclpy` کے فنڈامینٹلز کو سمجھیں**: `rclpy` کے بنیادی اجزاء اور وہ ROS 2 کے تصورات (نوڈس، ٹاپکس، سروسز، ایکشنز) سے کیسے ملتے ہیں کی وضاحت کریں۔
*   **`rclpy` نوڈس تیار کریں**: مختلف مواصلت کے نمونوں (پبلشر، سبسکرائبر، سروس کلائنٹ، سروس سرور) کے لیے `rclpy` کا استعمال کرتے ہوئے ROS 2 نوڈس لکھیں اور ڈیبگ کریں۔
*   **پائی تھون AI ایجنٹس کو یکجا کریں**: وضاحت کریں کہ `rclpy` ROS 2 روبوٹکس سسٹم کے ساتھ پائی تھون-مبنی AI اور مشین لرننگ الگوری دھم کو کیسے بے داغ طریقے سے یکجا کرتا ہے۔
*   **`rclpy` کے لیے URDF کو سمجھیں**: سمجھیں کہ `rclpy` URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) کے ساتھ کیسے تعامل کر سکتا ہے متحرک روبوٹ ماڈلنگ اور سیمولیشن کی تیاری کے لیے۔
*   **`robot_state_publisher` کا استعمال کریں**: URDF ماڈلز سے روبوٹ جوائنٹ اسٹیٹس کو براڈ کاسٹ کرنے میں `robot_state_publisher` کے کردار کی وضاحت کریں۔
*   **بنیادی URDF لوڈنگ/پارسنگ نافذ کریں**: URDF فائلز سے لوڈ کرنے اور معلومات نکالنے کے لیے سادہ `rclpy` اسکرپٹس تیار کریں۔
*   **ہیومنوائڈ کنٹرول کے لیے تیار کریں**: `rclpy` اور URDF کو استعمال کرتے ہوئے سیمولیشن یا جسمانی ماحول میں ہیومنوائڈ روبوٹس کنٹرول کرنے کے لیے شاملہ اقدامات کی وضاحت کریں۔

یہ سیکھنے کے نتائج اس باب کے مواد اور عملی مشق کی ہدایت کریں گے۔

## `rclpy`: پائی تھونک ROS 2 ترقی

`rclpy` ROS 2 کے لیے پائی تھون کلائنٹ لائبریری ہے، جو ROS 2 گراف کے ساتھ تعامل کرنے کے لیے ایک پائی تھونک انٹرفیس فراہم کرتا ہے۔ یہ ڈیولپرز کو ROS 2 نوڈس پائی تھون کا استعمال کرتے ہوئے لکھنے کی اجازت دیتا ہے، جو AI اور مشین لرننگ الگوری دھم کو یکجا کرنے کے لیے خاص طور پر فائدہ مند ہے کیونکہ پائی تھون کے سائنسی کمپیوٹنگ اور AI لائبریریز کا وسیع ماحول ہے۔

### بنیادی `rclpy` تصورات

`rclpy` ROS 2 کے بنیادی تصورات کو دہراتا ہے:

*   **نوڈس**: `rclpy` میں، ایک نوڈ عام طور پر ایک `Node` کلاس کے مثال کی نمائندگی کرتا ہے۔ آپ ایک `Node` بناتے ہیں، اس میں پبلشرز، سبسکرائبرز، سروسز، اور کلائنٹس شامل کرتے ہیں۔
*   **پبلشرز اور سبسکرائبرز**: ٹاپک مواصلت کو سیٹ اپ کرنے کے لیے `Node` کلاس کے `create_publisher()` اور `create_subscription()` طریقے استعمال کریں۔
*   **سروسز اور کلائنٹس**: درخواست-جواب کے نمونوں کے لیے `create_service()` اور `create_client()` استعمال کریں۔
*   **ایگزیکیوٹرز**: متعدد پبلشرز، سبسکرائبرز، کلائنٹس، اور سرورز سے کال بیکس کی انجام دہی کا نظم کریں۔

### مثال: سادہ `rclpy` پبلشر

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}" ')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### پائی تھون ایجنٹس اور ROS 2

`rclpy` AI ایجنٹس تیار کرنے کے لیے مثالی ہے جنہیں ROS 2 سسٹم کے ساتھ تعامل کرنے کی ضرورت ہے۔ مثال کے طور پر:

*   **ادراک ایجنٹس**: ایک پائی تھون نوڈ ایک کیمرہ ٹاپک کو سبسکرائب کر سکتا ہے، تصاویر کو گہری لرننگ ماڈل (مثلاً OpenCV، TensorFlow، PyTorch) کا استعمال کرتے ہوئے پروسیس کر سکتا ہے، اور دوسرے ٹاپک پر دریافت کردہ اشیاء کو پبلش کر سکتا ہے۔
*   **فیصلہ سازی ایجنٹس**: ایک ایجنٹ سینسر ڈیٹا کو سبسکرائب کر سکتا ہے، مضبوط لرننگ یا کوگنیٹو پلاننگ الگوری دھم کا استعمال کر سکتا ہے تاکہ روبوٹ کی اگلی حرکت کا فیصلہ کیا جا سکے، اور موٹر کنٹرول ٹاپکس پر کمانڈز کو پبلش کر سکتا ہے۔
*   **قدرتی زبان کے انٹرفیسز**: پائی تھون میں LLMs یا اسپیچ ریکوگنیشن (جیسے OpenAI وہسپر) کو یکجا کریں تاکہ انسانی کمانڈز کی تشریح کی جا سکے اور انہیں ROS 2 ایکشنز میں تبدیل کیا جا سکے۔

### URDF اور ہیومنوائڈز کے لیے `rclpy`

**URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ)** روبوٹ کے تمام عناصر کی وضاحت کے لیے ایک XML فارمیٹ ہے، بشمول اس کی وژوئل ظہور، تصادم کی خصوصیات، اور جڑان کی خصوصیات۔ ہیومنوائڈ روبوٹس کے لیے، URDF جوائنٹس، لنکس، اور ان کے تعلقات کی وضاحت کرتا ہے۔

`rclpy` نوڈس اکثر URDF کے ساتھ کئی طریقوں سے تعامل کرتے ہیں:

*   **`robot_state_publisher`**: ایک ROS 2 پیکیج جو روبوٹ کا URDF اور جوائنٹ اسٹیٹس (ایک `joint_states` ٹاپک پر پبلش کیا گیا) پڑھتا ہے اور پھر روبوٹ کی مکمل کنیمیٹک حالت (ٹرانسفارم) کو ROS 2 گراف میں براڈ کاسٹ کرتا ہے۔ یہ RViz جیسے ٹولز میں وژولائزیشن کے لیے اہم ہے۔
*   **متحرک روبوٹ کنفیگریشن**: پائی تھون اسکرپٹس URDF فائلز کو پارس کر سکتے ہیں تاکہ روبوٹ کے پیرامیٹرز کو متحرک طور پر کنفیگر کیا جا سکے، فارورڈ/انورس کنیمیٹکس کا حساب کیا جا سکے، یا مخصوص حرکت ٹریجکٹریز تیار کی جا سکیں۔

### مثال: پائی تھون کے ساتھ URDF لوڈ کرنا

```python
import xacro # pip install xacro
import rclpy
from rclpy.node import Node

class UrdfLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')
        self.declare_parameter('robot_description_file', '')
        robot_description_file = self.get_parameter('robot_description_file').value

        if not robot_description_file:
            self.get_logger().error('robot_description_file parameter is not set.')
            return

        try:
            # xacro فائل کو پروسیس کریں اگر لاگو ہو
            doc = xacro.process_file(robot_description_file)
            robot_description = doc.toprettyxml(indent='  ')
            self.get_logger().info('Successfully loaded and processed URDF/xacro file.')
            # آپ اب اس XML کو پارس کر سکتے ہیں یا اسے دوسرے ROS 2 ٹولز کے ساتھ استعمال کر سکتے ہیں
            # مثال کے طور پر، اسے ایک ٹاپک پر پبلش کریں یا اسے robot_state_publisher کنفیگر کرنے کے لیے استعمال کریں
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF/xacro: {e}')

def main(args=None):
    rclpy.init(args=args)
    urdf_loader = UrdfLoader()
    rclpy.spin_once(urdf_loader, timeout_sec=0.1)
    urdf_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**نوٹ**: اوپر کی مثال چلانے کے لیے، آپ کے پاس `xacro` انسٹال ہونا چاہیے (`pip install xacro`) اور ایک درست `.urdf` یا `.xacro` فائل `robot_description_file` پیرامیٹر کے ذریعہ فراہم کی جانی چاہیے۔

`rclpy` اور URDF کی یہ یک جہتی اعلیٰ ہیومنوائڈ روبوٹکس کے لیے بنیادی ہے، جو کنٹرول اور پیچیدہ روبوٹ کے رویوں کی درست سیمولیشن کو فعال کرتی ہے۔