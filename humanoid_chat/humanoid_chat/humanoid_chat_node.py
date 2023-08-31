import os
import threading
import rclpy
import rclpy.qos
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from humanoid_interface.msg import ChatResult
from .azure_speech import AzureSpeechService
from .iflytek_spark import SparkDesk
import sqlite3


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
        
        # initialize chat model
        self._chat_model = SparkDesk(
            os.environ.get('IFLYTEK_APP_ID'),
            os.environ.get('IFLYTEK_APP_KEY'),
            os.environ.get('IFLYTEK_APP_SECRET')
        )
        
        # Chatting state
        self._chatting = True
        self._chat_thread = threading.Thread(target=self._main_loop)
        self._chat_thread.start()
        
        # Ros2 interface
        self._chat_switch_server = self.create_service(SetBool, 'chat_switch', self._chat_switch_callback)
        self._chat_result_publisher = self.create_publisher(ChatResult, "chat_result", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._speak_subscription = self.create_subscription(String, "speak", self._speak_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
    
    def _speak_callback(self, msg: String):
        if not self._azure.is_speech_synthesising():
            self._azure.text_to_speech(msg.data)

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
    
    def _main_loop(self):
        self._azure.text_to_speech('请对我说小智')
        while self._chatting:
            # Detect keyword
            self.get_logger().info("Recognizing keyword.")
            while not self._azure.recognize_keyword(os.path.join(self._package_path, "xz.table")):
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
            
            # Search database
            if (preset_answer := self._serach_preset_answer(question)) is not None:
                # Using preset answer
                self.get_logger().info("Found preset answer in database.")
                print(preset_answer)
                self._chat_result_publisher.publish(
                    ChatResult(
                        question=question,
                        answer=preset_answer
                    )
                )
                self._azure.text_to_speech(preset_answer)
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
                            self._azure.text_to_speech(response[synthesis_ptr:sep_ptr])
                            synthesis_ptr = sep_ptr
                    prev_response = response
                print()
                self._chat_result_publisher.publish(
                    ChatResult(
                        question=question,
                        answer=prev_response
                    )
                )
                if prev_response[synthesis_ptr:]:
                    self._azure.text_to_speech(prev_response[synthesis_ptr:])
                self._azure.wait_speech_synthesising()


def main(args=None):
    rclpy.init(args=args)
    humanoid_chat_node = HumanoidChatNode()
    rclpy.spin(humanoid_chat_node)


if __name__ == '__main__':
    main()
