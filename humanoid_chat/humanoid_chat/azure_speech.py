import azure.cognitiveservices.speech as speechsdk
import queue
import time


class AzureSpeechService:
    def __init__(self, key, region, recognition_language='zh-CN', voice_name='zh-CN-XiaoxiaoNeural', microphone_device=None, speaker_device=None):
        # init speech recognizer
        speech_config = speechsdk.SpeechConfig(subscription=key, region=region)
        speech_config.speech_recognition_language = recognition_language
        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=microphone_device is None, device_name=microphone_device)
        self._speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
        self._speech_recognizer.recognizing.connect(self._recognizing)
        self._speech_recognizer.recognized.connect(self._recognized)
        self._speech_recognizer.session_stopped.connect(self._synthesis_stop)
        self._speech_recognizer.canceled.connect(self._synthesis_stop)
        self._speech_recognize_queue = queue.Queue()
        
        # init speech synthesizer
        speech_config = speechsdk.SpeechConfig(subscription=key, region=region)
        audio_config = speechsdk.audio.AudioOutputConfig(use_default_speaker=speaker_device is None, device_name=speaker_device)
        speech_config.speech_synthesis_voice_name = voice_name
        self._speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=audio_config)
        self._speech_synthesizer.synthesis_completed.connect(self._synthesis_stop)
        self._speech_synthesizer.synthesis_canceled.connect(self._synthesis_stop)
        self._speech_synthesizer.viseme_received.connect(self._viseme)
        self._speech_synthesis_result = None
        self._speech_synthesising = False
        self._viseme_callback = None
        self._synthesis_start_time = time.time()
        
        # init keyword model
        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=microphone_device is None, device_name=microphone_device)
        self._keyword_recognizer = speechsdk.KeywordRecognizer(audio_config=audio_config)
    
    def _synthesis_stop(self, evt):
        self._speech_synthesising = False
    
    def _recognize_stop(self, evt):
        self._speech_recognizer.stop_continuous_recognition()
    
    def _recognizing(self, evt):
        self._speech_recognize_queue.put(evt.result.text)
    
    def _viseme(self, evt: speechsdk.SpeechSynthesisVisemeEventArgs):
        if self._viseme_callback is not None:
            self._viseme_callback(evt.audio_offset / 10000000, evt.viseme_id)

    def _recognized(self, evt):
        self._speech_recognize_queue.put(evt.result.text)
        self._speech_recognize_queue.put(None)
        self._speech_recognizer.stop_continuous_recognition()
    
    def set_viseme_callback(self, callback):
        self._viseme_callback = callback

    def is_speech_synthesising(self):
        return self._speech_synthesising

    def wait_speech_synthesising(self):
        if self._speech_synthesis_result is not None:
            self._speech_synthesis_result.get()

    def speech_to_text(self):
        while not self._speech_recognize_queue.empty():
            response = self._speech_recognize_queue.get()
            if response is None:
                break
            yield response
            return
        result = self._speech_recognizer.start_continuous_recognition_async()
        while True:
            try:
                response = self._speech_recognize_queue.get(timeout=10.0)
            except queue.Empty:
                break
            if response is None:
                break
            yield response

    def text_to_speech(self, text):
        self._speech_synthesising = True
        self.wait_speech_synthesising()
        self._speech_synthesis_result = self._speech_synthesizer.speak_text_async(text)
    
    def recognize_keyword(self, model):
        result = self._keyword_recognizer.recognize_once_async(
            speechsdk.KeywordRecognitionModel(model)
        ).get()
        if result.reason == speechsdk.ResultReason.RecognizedKeyword:
            return True
        return False

    def stop_all(self):
        self._keyword_recognizer.stop_recognition()
        self._speech_synthesizer.stop_speaking()
