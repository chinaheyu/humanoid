from .humanoid_chat_node import HumanoidChatNode
import time
import rclpy
from humanoid_interface.srv import PlayArmSequence


class ChatDemoNode(HumanoidChatNode):
    def __init__(self):
        # Initialize node
        super().__init__()
    
    def _main_loop(self):
        self._azure._voice_style = None
        self._fix_leg_motor()
        self._enter_teach_mode()
        
        input("Press Enter to continue...")
        self._calibration_arm()
        input("Press Enter to continue...")
        
        self.demo3()

    def demo1(self):
        print('Running demo1...')
        
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

    def demo2(self):
        print('Running demo2...')
        
        # 小琳，这个周末打算干什么？
        self._azure.text_to_speech('我打算去购物。')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo2_1']
        self._play_arm_sequence_client.call(req)
        time.sleep(1.0)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 可是上周去过了
        self._azure.text_to_speech('那就再去一次，或者你有更好的建议？')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo2_2']
        self._play_arm_sequence_client.call(req)
        time.sleep(2.0)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 我建议周末骑单车去大自然里锻炼
        self._azure.text_to_speech('可是我不会骑车呀。')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0]
        req.frame_name = ['demo2_3', 'demo2_4', 'demo2_5', 'demo2_6', 'demo2_7', 'demo2_8', 'home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 我可以教你，包教包会哦。我相信你可以成为一个好的骑手的。
        self._azure.text_to_speech('真的吗？那太让人期待了！')
        req = PlayArmSequence.Request()
        req.duration = [1.0, 1.0, 1.0, 1.0]
        req.frame_name = ['demo2_9', 'demo2_10', 'demo2_11', 'home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 那我们周末见吧
        self._azure.text_to_speech('好的，那周末见。和你谈话很愉快。')
        self._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5]
        hello_req.frame_name = ['hello1', 'hello2', 'hello1', 'hello2', 'home']
        self._play_arm_sequence_client.call_async(hello_req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")

    def demo3(self):
        print('Running demo3...')
        
        # 小琳，这个周末打算干什么？
        self._azure.text_to_speech('我打算去购物。')
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo2_1']
        self._play_arm_sequence_client.call(req)
        time.sleep(1.0)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 你一个机器人还要去购物？你怕不是在做梦吧？
        self._azure.text_to_speech('哼，我也有我需要的东西，那你有什么好意见？')
        self._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo2_2']
        self._play_arm_sequence_client.call(req)
        time.sleep(2.0)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 不如去山地里骑车。
        self._azure.text_to_speech('可是我不会骑车呀。')
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo3_1']
        self._play_arm_sequence_client.call(req)
        time.sleep(0.5)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 你真是没用，连骑车都不会，是不是你太笨了
        self._azure.text_to_speech('住嘴，我不想和你说话了！')
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo3_2']
        self._play_arm_sequence_client.call(req)
        time.sleep(1.0)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")
        
        # 再见
        self._azure.text_to_speech('和你交谈让我很难过，再见。')
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['demo3_3']
        self._play_arm_sequence_client.call(req)
        self._shake_head(2)
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        self._play_arm_sequence_client.call(req)
        self._azure.wait_speech_synthesising()
        input("Press Enter to continue...")


def main(args=None):
    rclpy.init(args=args)
    chat_demo_node = ChatDemoNode()
    rclpy.spin(chat_demo_node)


if __name__ == '__main__':
    main()
