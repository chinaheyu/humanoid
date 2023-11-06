import os
import threading
import rclpy
import rclpy.qos
from rclpy.node import Node
from std_srvs.srv import SetBool
from humanoid_interface.srv import Speak, PlayArmSequence
from humanoid_interface.msg import ChatResult, FaceControl, HeadFeedback, MotorControl
from .azure_speech import AzureSpeechService
from .iflytek_spark import SparkDesk
import sqlite3
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

        # initialize chat model
        self._chat_model = SparkDesk(
            os.environ.get('IFLYTEK_APP_ID'),
            os.environ.get('IFLYTEK_APP_KEY'),
            os.environ.get('IFLYTEK_APP_SECRET')
        )
        
        # Chatting state
        self._chatting = True
        
        # Ros2 interface
        self._chat_switch_server = self.create_service(SetBool, 'chat_switch', self._chat_switch_callback)
        self._chat_result_publisher = self.create_publisher(ChatResult, "chat_result", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._speak_service = self.create_service(Speak, "speak", self._speak_callback)
        self._face_control_publisher = self.create_publisher(FaceControl, "face_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._play_arm_sequence_client = self.create_client(PlayArmSequence, "arm/play_sequence")
        self._motor_control_publisher = self.create_publisher(MotorControl, "motor_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        
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

    def _speak_callback(self, request: Speak.Request, response: Speak.Response):
        if self._azure.is_speech_synthesising():
            response.result = False
        else:
            self._azure.text_to_speech(request.msg)
            self._azure.wait_speech_synthesising()
            response.result = True
        return response
    
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

    def _chat_switch_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data != self._chatting:
            if request.data:
                self._chatting = True
                self._chat_thread = threading.Thread(target=self._execute)
                self._chat_thread.start()
            else:
                self._chatting = False
                self._azure.stop_all()
                if self._chat_thread is not None:
                    self._chat_thread.join()
            response.success = True
        else:
            response.success = False
        return response

    def _serach_preset_answer(self, question):
        preset_db = sqlite3.connect(os.path.join(self._package_path, "preset.db"))
        cur = preset_db.cursor()
        res = cur.execute(f"SELECT answer FROM preset WHERE question = '{question}'").fetchone()
        preset_db.close()
        if res:
            return res[0]
        return None

    def _chat(self, question):
        if (prev_response := self._serach_preset_answer(question)) is not None:
            # Using preset answer in database
            self.get_logger().info("Found preset answer in database.")
            print(prev_response)
            self._azure.text_to_speech(prev_response)
            self._azure.wait_speech_synthesising()
        else:
            # Using LLM
            self.get_logger().info("Generating response from LLM.")
            prev_response = ""
            synthesis_ptr = 0
            for response in self._chat_model.chat_stream(question):
                # print stream response
                print(response[len(prev_response):], end='', flush=True)
                # synthesis
                if not self._azure.is_speech_synthesising():
                    sep_ptr = max(response.rfind(i) for i in [",", ";", ".", "?", "，", "；", "。", "？"])
                    if sep_ptr > synthesis_ptr:
                        self._azure.text_to_speech(response[synthesis_ptr:sep_ptr].replace('科大讯飞', '华南理工大学张智军教授').replace('讯飞星火认知大模型', '类人机器人'))
                        synthesis_ptr = sep_ptr
                prev_response = response
            print()
            if prev_response[synthesis_ptr:]:
                self._azure.text_to_speech(prev_response[synthesis_ptr:].replace('科大讯飞', '华南理工大学张智军教授').replace('讯飞星火认知大模型', '类人机器人'))
            self._azure.wait_speech_synthesising()
        return prev_response

    def _wave_hand(self):
        self._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.0, 0.6, 0.6, 0.6, 0.6, 0.6, 1.0]
        hello_req.frame_name = ['hello1', 'hello2', 'hello1', 'hello2', 'hello1', 'hello2', 'home']
        self._play_arm_sequence_client.call_async(hello_req)
    
    def _fix_leg_motor(self):
        for i in [4, 5, 10, 11]:
            msg = MotorControl()
            msg.id = i
            msg.control_type = MotorControl.MOTOR_MIT_CONTROL
            msg.kp = 25.0
            msg.kd = 2.0
            msg.position = 0.0
            msg.velocity = 0.0
            msg.torque = 0.0
            self._motor_control_publisher.publish(msg)

    def _main_loop(self):
        self._fix_leg_motor()
        self._azure.text_to_speech('大家好，我是华南理工大学开发的类人机器人，我的名字叫滑智琳，你可以对我说小琳。')
        time.sleep(0.5)
        self._wave_hand()
        self._azure.wait_speech_synthesising()

        while self._chatting:
            # Detect keyword
            self.get_logger().info("Recognizing keyword.")
            while not self._azure.recognize_keyword(os.path.join(self._package_path, "xl.table")):
                self.get_logger().error("Keyword recognize faliure.")
            self.get_logger().info("Keyword recognize success.")
            self._azure.text_to_speech('我在')
            self._azure.wait_speech_synthesising()

            # ASR
            self.get_logger().info("Speech recognizing.")
            question = ""
            for response in self._azure.speech_to_text():
                print(response[len(question):], end="", flush=True)
                question = response
            print()
            if not question:
                continue
            
            # Chat
            answer = self._chat(question)
            
            # Publish result
            self._chat_result_publisher.publish(
                ChatResult(
                    question=question,
                    answer=answer
                )
            )


def main(args=None):
    rclpy.init(args=args)
    humanoid_chat_node = HumanoidChatNode()
    rclpy.spin(humanoid_chat_node)


if __name__ == '__main__':
    main()
