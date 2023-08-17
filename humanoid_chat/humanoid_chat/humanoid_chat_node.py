import os
import threading
import rclpy
import rclpy.qos
from rclpy.node import Node
from std_srvs.srv import SetBool
from humanoid_interface.msg import ChatResult
from .azure_speech import AzureSpeechService
from .iflytek_spark import SparkDesk


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
        self.chatting = True
        self.chat_thread = threading.Thread(target=self._main_loop)
        self.chat_thread.start()
        
        # Ros2 interface
        self.chat_switch_server = self.create_service(SetBool, 'chat_switch', self._chat_switch_callback)
        self.chat_result_publisher = self.create_publisher(ChatResult, "chat_result", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
    
    def _chat_switch_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data != self.chatting:
            if request.data:
                self.chatting = True
                self.chat_thread = threading.Thread(target=self._execute)
                self.chat_thread.start()
            else:
                self.chatting = False
                self._azure.stop_all()
                if self.chat_thread is not None:
                    self.chat_thread.join()
            response.success = True
        else:
            response.success = False
        return response
    
    def _main_loop(self):
        self._azure.text_to_speech('请对我说小智')
        while self.chatting:
            # Detect keyword
            self.get_logger().info("Recognizing keyword.")
            while not self._azure.recognize_keyword(os.path.join(self._package_path, "xz.table")):
                self.get_logger().error("Keyword recognize faliure.")
            self.get_logger().info("Keyword recognize success.")
            self._azure.text_to_speech('我在')
            self._azure.wait_speech_synthesising()

            # ASR
            question = ""
            for response in self._azure.speech_to_text():
                print(response[len(question):], end="", flush=True)
                question = response
            print()
            if not question:
                continue

            # Chat and tts
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
            self.chat_result_publisher.publish(
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
