import os
import threading
import rclpy
import rclpy.qos
from rclpy.node import Node
from humanoid_interface.srv import PlayArmSequence
from humanoid_interface.msg import ChatResult, FaceControl, HeadFeedback, MotorControl, NeckControl
from .azure_speech import AzureSpeechService
import time
import queue
import random
from std_srvs.srv import SetBool, Empty


class HumanoidChatNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('humanoid_chat')
        
        # get package path
        self._package_path = os.path.dirname(os.path.abspath(__file__))
        
        # initialize azure speech service
        self._azure = AzureSpeechService(
            os.environ.get('AZURE_SPEECH_KEY'),
            os.environ.get('AZURE_SPEECH_REGION'),
            microphone_device="sysdefault:CARD=DELI14870",
            speaker_device="sysdefault:CARD=DELI14870"
        )
        self._viseme_queue = queue.Queue()
        self._viseme_thread = threading.Thread(target=self._viseme_thread_callback)
        self._viseme_thread.start()
        self._azure.set_viseme_callback(self._viseme_callback)
        
        # Ros2 interface
        self._face_control_publisher = self.create_publisher(FaceControl, "face_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._play_arm_sequence_client = self.create_client(PlayArmSequence, "arm/play_sequence")
        self._neck_control_publisher = self.create_publisher(NeckControl, "neck_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._teach_mode_client = self.create_client(SetBool, "arm/teach_mode")
        self._calibration_client = self.create_client(Empty, "arm/calibration")

        # head feedback
        self._head_feedback_msg = HeadFeedback()
        self._head_feedback_subscriber = self.create_subscription(HeadFeedback, "head_feedback", self._head_feedback_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        
        # blink timer
        self._blink_thread = None
        self._blink_timer = self.create_timer(5.0, self._blink_timer_callback)
        
        # start chat loop
        self._chat_thread = threading.Thread(target=self._main_loop)
        self._chat_thread.start()
        
        self._gesture_on = False
        self._gesture_thread = threading.Thread(target=self._gesture_thread_callback)
        self._gesture_thread.start()
    
    def _head_feedback_callback(self, msg: HeadFeedback):
        self._head_feedback_msg = msg

    def _blink_thread_callback(self):
        msg = FaceControl()
        # eyelid down
        msg.pulse_width = self._head_feedback_msg.pulse_width
        msg.pulse_width[FaceControl.SERVO_LEFT_EYELID_UP_DOWN] = 905
        msg.pulse_width[FaceControl.SERVO_RIGHT_EYELID_UP_DOWN] = 2088
        self._face_control_publisher.publish(msg)
        # wait
        time.sleep(0.1)
        # eyelid open
        msg.pulse_width = self._head_feedback_msg.pulse_width
        msg.pulse_width[FaceControl.SERVO_LEFT_EYELID_UP_DOWN] = 1500
        msg.pulse_width[FaceControl.SERVO_RIGHT_EYELID_UP_DOWN] = 1500
        self._face_control_publisher.publish(msg)

    def _blink_timer_callback(self):
        # wait previous thread finish
        if self._blink_thread is not None:
            self._blink_thread.join()
        # start new thread
        self._blink_thread = threading.Thread(target=self._blink_thread_callback)
        self._blink_thread.start()
    
    def _viseme_thread_callback(self):
        viseme_to_chin = {
            0: 910,
            1: 1100,
            2: 1100,
            3: 1000,
            4: 1000,
            5: 1000,
            6: 980,
            7: 980,
            8: 980,
            9: 1100,
            10: 1100,
            11: 1100,
            12: 1000,
            13: 1100,
            14: 1000,
            15: 980,
            16: 980,
            17: 1000,
            18: 980,
            19: 1000,
            20: 1100,
            21: 910
        }
        current_time = 0.0
        while rclpy.ok():
            # FIXME: sync viseme with audio
            timestamp, viseme_id = self._viseme_queue.get()
            if current_time < timestamp:
                time.sleep(timestamp - current_time)
            else:
                time.sleep(timestamp)
            current_time = timestamp
            if viseme_id in viseme_to_chin:
                msg = FaceControl()
                msg.pulse_width = self._head_feedback_msg.pulse_width
                msg.pulse_width[FaceControl.SERVO_CHIN_UP_DOWN] = viseme_to_chin[viseme_id]
                self._face_control_publisher.publish(msg)

    def _viseme_callback(self, timestamp, viseme_id):
        self._viseme_queue.put((timestamp, viseme_id))

    def _wave_hand(self):
        self._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.0, 0.6, 0.6, 0.6, 0.6, 0.6, 1.0]
        hello_req.frame_name = ['hello1', 'hello2', 'hello1', 'hello2', 'hello1', 'hello2', 'home']
        self._play_arm_sequence_client.call_async(hello_req)
    
    def _nod_head(self, times=1):
        for _ in range(times):
            msg = NeckControl()
            msg.pitch_velocity = 0.3
            msg.yaw_angle = 0.25
            msg.yaw_max_velocity = 3.14
            self._neck_control_publisher.publish(msg)
            time.sleep(0.7)
            msg = NeckControl()
            msg.pitch_velocity = -0.3
            msg.yaw_angle = 0.25
            msg.yaw_max_velocity = 3.14
            self._neck_control_publisher.publish(msg)
            time.sleep(0.7)
        msg = NeckControl()
        msg.pitch_velocity = 0.3
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        self._neck_control_publisher.publish(msg)
        time.sleep(0.2)
        msg = NeckControl()
        msg.pitch_velocity = 0.0
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        self._neck_control_publisher.publish(msg)
    
    def _calibration_arm(self):
        msg = Empty.Request()
        self._calibration_client.wait_for_service()
        self._calibration_client.call(msg)
        msg = SetBool.Request()
        msg.data = False
        self._teach_mode_client.wait_for_service()
        self._teach_mode_client.call(msg)
    
    def _gesture_thread_callback(self):
        gesture_list = ['talk1', 'talk2', 'talk3', 'talk4']
        running_gesture = False
        current_gesture = None
        while rclpy.ok():
            if self._gesture_on:
                running_gesture = True
            else:
                if running_gesture:
                    running_gesture = False
                    msg = PlayArmSequence.Request()
                    msg.duration = [2.0]
                    msg.frame_name = ['home']
                    self._play_arm_sequence_client.wait_for_service()
                    self._play_arm_sequence_client.call(msg)
            if running_gesture:
                msg = PlayArmSequence.Request()
                msg.duration = [2.0]
                while (random_gesture := random.choice(gesture_list)) == current_gesture:
                    random_gesture = random.choice(gesture_list)
                msg.frame_name = [random_gesture]
                self._play_arm_sequence_client.wait_for_service()
                self._play_arm_sequence_client.call(msg)
            time.sleep(1.0)

    def _main_loop(self):
        self._azure.text_to_speech('开始校准手臂电机，请让双臂自然下垂。')
        input("Press Enter to continue...")
        self._calibration_arm()
        self._azure.wait_speech_synthesising()
        
        self._azure.text_to_speech('大家好！我是华南理工大学超级机器人研究院张智军教授团队研发的人形机器人，我的名字叫滑智琳，你可以叫我小琳。')
        time.sleep(0.5)
        self._wave_hand()
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        self._azure.text_to_speech('上知天文下知地理。')
        self._nod_head(1)
        self._azure.wait_speech_synthesising()
        self._azure.text_to_speech('多才多艺。')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [1.0]
        req.frame_name = ['extend']
        self._play_arm_sequence_client.call(req)
        time.sleep(1.0)
        req.duration = [1.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._gesture_on = True
        self._azure.wait_speech_synthesising()
        self._azure.text_to_speech('我可以被用于，家庭服务，办公辅助，酒店前台，场馆导览等场景应用，还可以作为优质的科研平台。')
        self._azure.wait_speech_synthesising()
        self._gesture_on = False
        input("Press Enter to continue...")
        
        self._gesture_on = True
        self._azure.text_to_speech('超级机器人研究院是由华南理工大学与黄浦区合作创办的科研平台。是面向国家重大战略需求的科研创新高地，也是人才培养的摇篮。致力于校企合作成果转化，孵化高质量、高水平科技企业，打造校地合作新典范。在未来将成为粤港澳大湾区的科技创新领军基地。还有什么想要知道的吗？')
        self._azure.wait_speech_synthesising()
        self._gesture_on = False
        input("Press Enter to continue...")
        
        self._azure.text_to_speech('感谢你对我的喜爱。')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [1.5]
        req.frame_name = ['shake_hand']
        self._play_arm_sequence_client.call(req)
        time.sleep(2.0)
        req.duration = [1.5]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        self._azure.text_to_speech('下次见。')
        self._wave_hand()
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")


def main(args=None):
    rclpy.init(args=args)
    humanoid_chat_node = HumanoidChatNode()
    rclpy.spin(humanoid_chat_node)


if __name__ == '__main__':
    main()
