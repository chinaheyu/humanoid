from .humanoid_chat_node import HumanoidChatNode
import time
import rclpy
from humanoid_interface.srv import PlayArmSequence


class ChatDemoNode(HumanoidChatNode):
    def __init__(self):
        # Initialize node
        super().__init__()

    def _main_loop(self):
        self._fix_leg_motor()
        self._enter_teach_mode()
        
        self._azure.text_to_speech('开始校准手臂电机，请让双臂自然下垂。')
        input("Press Enter to continue...")
        self._calibration_arm()
        
        self._azure.text_to_speech('大家好！我是华南理工大学超级机器人研究院张智军教授团队研发的人形机器人，我的名字叫滑智琳，你也可以叫我小琳。')
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
    chat_demo_node = ChatDemoNode()
    rclpy.spin(chat_demo_node)


if __name__ == '__main__':
    main()
