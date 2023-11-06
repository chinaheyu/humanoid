import os
import threading
import rclpy
import rclpy.qos
from rclpy.node import Node
from humanoid_interface.srv import PlayArmSequence
from humanoid_interface.msg import FaceControl, HeadFeedback
from .azure_speech import AzureSpeechService
import time
import queue


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
        
        # head feedback
        self._head_feedback_msg = HeadFeedback()
        self._head_feedback_subscriber = self.create_subscription(HeadFeedback, "head_feedback", self._head_feedback_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        
        # blink timer
        self._blink_thread = None
        self._blink_timer = self.create_timer(5.0, self._blink_timer_callback)
        
        # start chat loop
        self._chat_thread = threading.Thread(target=self._main_loop)
        self._chat_thread.start()
    
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

    def _main_loop(self):
        self._azure.text_to_speech('大家好，我是华南理工大学开发的类人机器人，我的名字叫滑智琳，你可以对我说小琳。')
        time.sleep(0.5)
        self._wave_hand()
        self._azure.wait_speech_synthesising()

        for answer in [
            '我在',
            '我来自华南理工大学自动化科学与工程学院。',
            '上知天文下知地理，懂的都懂。',
            '华南理工大学是位于广东省广州市的一所全国重点大学，被列为国家“双一流”、“211工程”和“985工程”。',
            '华南理工大学张智军教授和他的团队创造了我。',
            '我可以通过自然语言交互的方式，为您提供包括语言理解、问答、推理等各类认知智能服务。',
            '我的兴趣爱好挺广泛的，主要是阅读书籍。',
            '可以呀。'
        ]:
            print(f'Next answer: {answer}')
            input("Press Enter to continue...")
            self._azure.text_to_speech(answer)
            self._azure.wait_speech_synthesising()


def main(args=None):
    rclpy.init(args=args)
    humanoid_chat_node = HumanoidChatNode()
    rclpy.spin(humanoid_chat_node)


if __name__ == '__main__':
    main()
