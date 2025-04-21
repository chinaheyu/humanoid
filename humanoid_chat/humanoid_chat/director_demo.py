import rclpy.executors
from .humanoid_chat_node import HumanoidChatNode
import time
import rclpy
from humanoid_interface.msg import NeckControl, FaceControl
from humanoid_interface.srv import PlayArmSequence, DoAction, GetActionList
from typing import Type, Union, List
import queue
import numpy as np
import math


class DirectorDemoNode(HumanoidChatNode):
    def __init__(self, action_list: List[Type['DirectorActionBase']]):
        super().__init__('director_demo_node')
        self._action_list = action_list
        self._do_action_service = self.create_service(DoAction, 'do_action', self._do_action_callback)
        self._get_action_list_service = self.create_service(GetActionList, 'get_action_list', self._get_action_list_callback)
        self._current_action = None
        self._calibrated = False
        self._action_queue = queue.Queue()
    
    def _get_action_list_callback(self, request: GetActionList.Request, response: GetActionList.Response) -> GetActionList.Response:
        response.actions = [str(x) for x in self._action_list]
        return response

    def _do_action_callback(self, request: DoAction.Request, response: DoAction.Response) -> DoAction.Response:
        if not self._calibrated:
            response.result = False
            return response
        action = next((x for x in self._action_list if x == request.action), None)
        if action is None:
            response.result = False
            return response
        self._action_queue.put(action)
        response.result = True
        return response
    
    def _do_action(self, action: Type['DirectorActionBase']) -> None:
        if self._current_action is None:
            action.call_init(self)
            self._current_action = action
        elif self._current_action == action:
            if self._current_action._finished:
                self._current_action.call_init(self)
            else:
                self._current_action.call_run(self)
        elif action in self._current_action._next_action:
            if self._current_action._finished:
                action.call_run(self)
            else:
                action.call_init(self)
            self._current_action = action
        else:
            action.call_init(self)
            self._current_action = action

    def _main_loop(self):
        self._fix_leg_motor()
        self._enter_teach_mode()

        self._azure.text_to_speech('开始校准手臂电机，请让双臂自然下垂，完成校准后请对我说小琳。')
        self._azure.wait_speech_synthesising()
        self._detect_keyword()
        self._calibration_arm()
        self._azure.text_to_speech('已完成校准。')
        self._azure.wait_speech_synthesising()
        self._calibrated = True

        while rclpy.ok():
            try:
                action = self._action_queue.get()
            except queue.Empty:
                pass
            else:
                self._do_action(action)

    def add_action(self, action: Type['DirectorActionBase']):
        if action in self._action_list:
            raise ValueError('Action already exists.')
        self._action_list.append(action)


class DirectorActionBase:
    def __init__(self, name: str, parent_action: Union[Type['DirectorActionBase'], None] = None) -> None:
        self._name = name
        self._next_action = []
        self._finished = False
        if parent_action is not None:
            parent_action._next_action.append(self)

    def __eq__(self, v: object) -> bool:
        if isinstance(v, DirectorActionBase):
            return self._name == v._name
        if isinstance(v, str):
            return self._name == v
        return NotImplemented

    def __str__(self) -> str:
        return self._name

    def call_init(self, node: DirectorDemoNode) -> None:
        node.get_logger().info(f'Init action: {self._name}')
        self.init(node)
        self._finished = False
    
    def call_run(self, node: DirectorDemoNode) -> None:
        node.get_logger().info(f'Run action: {self._name}')
        self.run(node)
        self._finished = True

    def init(self, node: DirectorDemoNode) -> None:
        node._gesture_on = False
        node._blink_on = False
        node._eye_move_on = False
        msg = NeckControl()
        msg.pitch_velocity = 0.0
        msg.yaw_angle = 0.15
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['home']
        node._play_arm_sequence_client.call(req)

    def run(self, node: DirectorDemoNode) -> None:
        pass


class WaveHandAction(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('wave_hand', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._wave_hand()


class RollingEyesAction(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('rolling_eyes', parent_action)
    
    def init(self, node: DirectorDemoNode) -> None:
        node._blink_on = False

    def run(self, node: DirectorDemoNode) -> None:
        node._rolling_eyes(repeat=2)
        node._blink_on = True


class SelfIntroductuion(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('self_introduction', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('大家好！我是华南理工大学张智军教授团队研发的人形机器人，我的名字叫滑智琳，你也可以叫我小琳。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class IntroduceSCUT(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('introduce_scut', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('华南理工大学坐落广州，是教育部直属的国家重点大学。源自1918年的广东省立第一甲种工业学校，史称“红色甲工”。学校实力雄厚，14个学科进入ESI全球排名前1‰，其中工程学、材料科学、化学等领域处于全球领先。毕业生就业率高，涌现科技骨干和企业领袖，是工程师和企业家的摇篮。拥有众多国家级科研平台，科研实力居全国前列。学校秉承“博学慎思 明辨笃行”的校训，弘扬“厚德尚学 自强不息 务实创新 追求卓越”的精神。以“双一流”建设和广州国际校区建设为双引擎，全力推进“学术华工”“开放华工”“善治华工”“幸福华工”“大美华工”建设。为实现中国梦，向着中国特色、世界一流大学的目标奋勇前进，勇当粤港澳大湾区高等教育发展排头兵，努力为实现中华民族伟大复兴的中国梦贡献华工智慧和华工力量。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ShakeHandAction(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('shake_hand', parent_action)
    
    def init(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [1.5]
        req.frame_name = ['shake_hand1']
        node._play_arm_sequence_client.call(req)

    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        req.frame_name = ['shake_hand2', 'shake_hand1', 'shake_hand2', 'shake_hand1', 'shake_hand2', 'shake_hand1']
        node._play_arm_sequence_client.call(req)


class GrabBlocksDemoAction(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('grab_blocks', parent_action)
    
    def init(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['grab1']
        node._play_arm_sequence_client.call(req)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['grab2']
        node._play_arm_sequence_client.call(req)


class YeahAction(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('yeah', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        req = PlayArmSequence.Request()
        req.duration = [2.0]
        req.frame_name = ['yeah']
        node._play_arm_sequence_client.call(req)


class HospitalDemoAction1(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo1', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        msg = NeckControl()
        for j in np.arange(0.15, -0.37, -0.036):
            msg.pitch_velocity = 0.0
            msg.yaw_angle = j
            msg.yaw_max_velocity = 3.14
            node._neck_control_publisher.publish(msg)
            time.sleep(0.02)


class HospitalDemoAction2(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo2', parent_action)
    
    def init(self, node: DirectorDemoNode) -> None:
        msg = NeckControl()
        msg.pitch_velocity = 0.0
        msg.yaw_angle = -0.37
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
    
    def run(self, node: DirectorDemoNode) -> None:
        msg = NeckControl()
        for j in np.arange(-0.37, 0.15, 0.036):
            msg.pitch_velocity = 0.0
            msg.yaw_angle = j
            msg.yaw_max_velocity = 3.14
            node._neck_control_publisher.publish(msg)
            time.sleep(0.02)


class HospitalDemoAction3(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo3', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech("你好。")
        node._wave_hand()
        node._azure.wait_speech_synthesising()


class HospitalDemoAction4(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo4', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.5, 1.5, 1.5, 1.5]
        hello_req.frame_name = ['hospital_demo_1', 'hospital_demo_2', 'hospital_demo_3', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech("有什么我可以帮助您的吗？")
        node._azure.wait_speech_synthesising()


class HospitalDemoAction5(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo5', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech("当然可以。")
        node._nod_head(2)
        node._azure.wait_speech_synthesising()


class HospitalDemoAction6(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo6', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        hello_req.frame_name = ['hospital_demo_4', 'hospital_demo_5', 'hospital_demo_6', 'hospital_demo_7', 'hospital_demo_6', 'hospital_demo_7', 'hospital_demo_8', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech("请您靠近一些。")
        node._azure.wait_speech_synthesising()


class HospitalDemoAction7(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo7', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech("正在检查您的体温。")
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1700, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1700, 1500, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        node._azure.wait_speech_synthesising()


class HospitalDemoAction8(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo8', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0]
        hello_req.frame_name = ['hospital_demo_13']
        node._play_arm_sequence_client.call(hello_req)
        node._azure.text_to_speech("37.5度，看上去有一些低烧，请问还有哪里不舒服呢？")
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0]
        hello_req.frame_name = ['home']
        node._play_arm_sequence_client.call(hello_req)
        node._azure.wait_speech_synthesising()


class HospitalDemoAction9(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo9', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['hospital_demo_9', 'hospital_demo_10', 'hospital_demo_11', 'hospital_demo_12', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech("之前有没有看过医生？")
        node._azure.wait_speech_synthesising()


class HospitalDemoAction10(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo10', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['show_direction1', 'show_direction2', 'show_direction3', 'show_direction4', 'show_direction5', 'show_direction6', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech("了解了，根据您的描述，您可能是感冒或者过敏引起的喉咙痛。我建议您先去挂个内科或者耳鼻喉科的号，让医生给您检查一下，看看具体是什么原因。")
        node._azure.wait_speech_synthesising()


class HospitalDemoAction11(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo11', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['hospital_demo_1', 'hospital_demo_2', 'hospital_demo_3', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech("在就诊过程中，请佩戴口罩，保持社交距离，避免拥挤的地方。同时，多喝水，保持充足的休息，增强身体抵抗力。如有需要，请随时联系我们。")
        node._azure.wait_speech_synthesising()


class HospitalDemoAction12(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('hospital_demo12', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech("不客气，祝您就诊顺利，早日康复！")
        node._wave_hand()
        node._azure.wait_speech_synthesising()


class LXRDemo1(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('lxr_demo1', parent_action)

    def init(self, node: DirectorDemoNode) -> None:
        node._eye_move_on = False
        msg = NeckControl()
        msg.pitch_velocity = -0.3
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
        time.sleep(1.0)
        msg = NeckControl()
        msg.pitch_velocity = 0.3
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
        time.sleep(0.2)
        msg.pitch_velocity = 0.0
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)

    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0]
        hello_req.frame_name = ['lxr_1', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech('将机器狗改造为导盲犬的想法是很有创意和潜力的。')
        node._azure.wait_speech_synthesising()
        
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
        hello_req.frame_name = ['lxr_2', 'lxr_3', 'lxr_4', 'lxr_3', 'lxr_2', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech('虽然目前机器狗的技术还在发展中，并且在复杂环境中实现导盲功能仍然面临挑战。')
        node._azure.wait_speech_synthesising()
        
        time.sleep(1)

        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0]
        hello_req.frame_name = ['lxr_1', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech('但这个想法可以作为一个创新的起点。')
        node._azure.wait_speech_synthesising()


class LXRDemo2(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('lxr_demo2', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('目前常用于障碍物检测的传感器有：（1）激光雷达：它可以提供高精度的障碍物检测和建图能力，广泛用于导航和避障。（2）超声波传感器：它适用于近距离的障碍物检测，但精度和范围有限。（3）红外传感器：它常用于近距离的障碍物检测，但在阳光强烈的环境中可能受到干扰。（4）视觉传感器：通过计算机视觉算法，摄像头可以捕捉环境图像，并进行物体检测和跟踪。这种传感器可以提供丰富的信息，但对于复杂场景和光照条件有一定的挑战。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class LXRDemo3(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('lxr_demo3', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('MATLAB和Python是不同的编程语言，它们具有不同的语法和功能。因此，在转换过程中可能需要进行一些适应性调整和改进。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False
        
        node._azure.text_to_speech('请您把程序上传给我。')
        node._azure.wait_speech_synthesising()


class LXRDemo4(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('lxr_demo4', parent_action)

    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['lxr_5', 'lxr_6', 'lxr_5', 'lxr_6', 'home']
        node._play_arm_sequence_client.call_async(hello_req)
        node._azure.text_to_speech('我现在学习一下你的这段Matlab程序，然后再帮你转化成Python程序。')
    
        msg = NeckControl()
        msg.pitch_velocity = 0.3
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
        time.sleep(0.2)
        msg.pitch_velocity = 0.0
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)

        with node._eye_mutex:
            for _ in range(4):
                st = time.time()
                while (t := time.time() - st) < 2.0:
                    node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_LEFT_RIGHT] = int(1500 + 400 * math.cos(2 * math.pi * t / 2.0))
                    node._face_control_publisher.publish(node._face_control_msg)
                    time.sleep(0.02)
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_LEFT_RIGHT] = 1500
            node._face_control_publisher.publish(node._face_control_msg)

        node._azure.wait_speech_synthesising()
        msg.pitch_velocity = -0.3
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)
        time.sleep(0.2)
        msg.pitch_velocity = 0.0
        msg.yaw_angle = 0.25
        msg.yaw_max_velocity = 3.14
        node._neck_control_publisher.publish(msg)


class ChineseMedicineDemo1(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo1', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('欢迎来到，中医健康体质判断平台。我是你的私人中医医疗机器人小琳。如果你准备好了，请对我说：小琳')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo2(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo2', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech('好的，让我们进入面向扫描环节。请看着我的眼睛。')
        node._azure.wait_speech_synthesising()


class ChineseMedicineDemo3(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo3', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1700, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1700, 1500, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)


class ChineseMedicineDemo4(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo4', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech('好的，我们进入舌苔扫描环节。请您伸出您的舌头。')
        node._azure.wait_speech_synthesising()


class ChineseMedicineDemo5(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo5', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1700, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1700, 1500, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1500, 1300, -10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)
        for i in range(1300, 1500, 10):
            node._face_control_msg.pulse_width[FaceControl.SERVO_EYES_UP_DOWN] = i
            node._face_control_publisher.publish(node._face_control_msg)
            time.sleep(0.02)


class ChineseMedicineDemo6(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo6', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._azure.text_to_speech('好的，我们进入脉象诊断环节。请您将手放在桌面的框里。')
        node._azure.wait_speech_synthesising()


class ChineseMedicineDemo7(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo7', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['cmdemo1', 'cmdemo2', 'cmdemo3', 'cmdemo4', 'cmdemo5']
        node._play_arm_sequence_client.call(hello_req)


class ChineseMedicineDemo8(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo8', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._play_arm_sequence_client.wait_for_service()
        hello_req = PlayArmSequence.Request()
        hello_req.duration = [2.0, 2.0, 2.0, 2.0]
        hello_req.frame_name = ['cmdemo6', 'cmdemo7', 'cmdemo8', 'cmdemo9']
        node._play_arm_sequence_client.call(hello_req)


class ChineseMedicineDemo9(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo9', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('好的，经过初步的观察，我想了解一下您平时的身体状况。首先，请问您是否容易感到疲乏呢？比如说，稍微活动一下或者爬上爬下就感到累？')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo10(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo10', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('好的，了解了。那请问您有没有经常觉得气短，没有感觉接不上气。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo11(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo11', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('嗯，好的。再请问您，您说话声音会不会感觉低弱无力，像没有力气那样。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo12(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo12', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('好的，了解了。请问您最近是否有感冒？')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo13(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo13', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('好的，了解了。请问您的手脚容不容易发凉。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo14(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo14', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('好的，了解了。请问您的睡眠质量怎么样？')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo15(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo15', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('本次体质检测已完成，您的体质是痰湿质，感谢你的参与。报告已生成，听去医生那边领取。还有什么问题需要问我的吗？')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo16(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo16', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('痰湿体质在肥胖人群中比较普遍，它是由于水液内停而痰湿凝聚，以粘滞重浊为主要特征的体质状态。表现为：体形肥胖，腹部肥满松软；面部皮肤油脂较多，多汗且黏，胸闷，痰多；或面色淡黄而暗，眼胞微浮，容易困倦；平素舌体胖大，苔白腻，口黏腻或甜，喜食肥甘甜黏；身重不爽，大便正常或不实，小便不多或微浊 ，脉诊比较滑；')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


class ChineseMedicineDemo17(DirectorActionBase):
    def __init__(self, parent_action: Union[Type[DirectorActionBase], None] = None) -> None:
        super().__init__('chinese_medicine_demo17', parent_action)
    
    def run(self, node: DirectorDemoNode) -> None:
        node._gesture_on = True
        node._azure.text_to_speech('管住嘴迈开腿，化痰祛湿为主；饮食清淡，容易消化；慎食肥甘厚味，少喝酒勿过饱；特别提醒：痰湿体质夏天容易依赖空调，可以经常喝一杯生姜茶，会出汗畅通，情绪平稳，明显增强耐热力，可以改善痰湿体质。')
        node._azure.wait_speech_synthesising()
        node._gesture_on = False


def main(args=None):
    rclpy.init(args=args)
    
    action_list = []
    
    action_list.append(DirectorActionBase('home'))
    action_list.append(WaveHandAction())
    action_list.append(RollingEyesAction())
    action_list.append(SelfIntroductuion())
    action_list.append(IntroduceSCUT())
    action_list.append(GrabBlocksDemoAction())
    action_list.append(ShakeHandAction())
    action_list.append(YeahAction())
    
    action_list.append(HospitalDemoAction1())
    action_list.append(HospitalDemoAction2(action_list[-1]))
    action_list.append(HospitalDemoAction3(action_list[-1]))
    action_list.append(HospitalDemoAction4(action_list[-1]))
    action_list.append(HospitalDemoAction5(action_list[-1]))
    action_list.append(HospitalDemoAction6(action_list[-1]))
    action_list.append(HospitalDemoAction7(action_list[-1]))
    action_list.append(HospitalDemoAction8(action_list[-1]))
    action_list.append(HospitalDemoAction9(action_list[-1]))
    action_list.append(HospitalDemoAction10(action_list[-1]))
    action_list.append(HospitalDemoAction11(action_list[-1]))
    action_list.append(HospitalDemoAction12(action_list[-1]))
    
    action_list.append(LXRDemo1())
    action_list.append(LXRDemo2(action_list[-1]))
    action_list.append(LXRDemo3(action_list[-1]))
    action_list.append(LXRDemo4(action_list[-1]))
    
    action_list.append(ChineseMedicineDemo1())
    action_list.append(ChineseMedicineDemo2(action_list[-1]))
    action_list.append(ChineseMedicineDemo3(action_list[-1]))
    action_list.append(ChineseMedicineDemo4(action_list[-1]))
    action_list.append(ChineseMedicineDemo5(action_list[-1]))
    action_list.append(ChineseMedicineDemo6(action_list[-1]))
    action_list.append(ChineseMedicineDemo7(action_list[-1]))
    action_list.append(ChineseMedicineDemo8(action_list[-1]))
    action_list.append(ChineseMedicineDemo9(action_list[-1]))
    action_list.append(ChineseMedicineDemo10(action_list[-1]))
    action_list.append(ChineseMedicineDemo11(action_list[-1]))
    action_list.append(ChineseMedicineDemo12(action_list[-1]))
    action_list.append(ChineseMedicineDemo13(action_list[-1]))
    action_list.append(ChineseMedicineDemo14(action_list[-1]))
    action_list.append(ChineseMedicineDemo15(action_list[-1]))
    action_list.append(ChineseMedicineDemo16(action_list[-1]))
    action_list.append(ChineseMedicineDemo17(action_list[-1]))

    director_demo_node = DirectorDemoNode(action_list)
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(director_demo_node)
    executor.spin()


if __name__ == '__main__':
    main()
