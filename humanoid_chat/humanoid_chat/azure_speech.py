from typing import Optional, Iterable, Callable, Tuple
import azure.cognitiveservices.speech as speechsdk
import queue
import string
import subprocess
import threading
import time
import hashlib
import os
import shutil
import json


ssml_template = string.Template("""\
<speak version="1.0"
    xmlns="http://www.w3.org/2001/10/synthesis"
    xmlns:mstts="https://www.w3.org/2001/mstts" xml:lang="${language}">
    <voice name="${voice_name}">
        <mstts:express-as style="${voice_style}">${text}</mstts:express-as>
    </voice>
</speak>\
""")


class SynthesizeResult:
    def __init__(self, device: str, audio_data: bytes, viseme_data: Iterable[Tuple[float, int]], viseme_callback: Optional[Callable[[int], None]]) -> None:
        self.device = device
        self.audio_data = audio_data
        self.viseme_data = viseme_data
        self.viseme_callback = viseme_callback
        self.aplay_handle: Optional[subprocess.Popen] = None
        self.cancel_event = threading.Event()
        self.call_viseme_thread: Optional[threading.Thread] = None

    def call_viseme(self):
        start_time = time.time()
        for v in self.viseme_data:
            current_time = time.time() - start_time
            if current_time < v[0]:
                if self.cancel_event.wait(v[0] - current_time):
                    break
            if not self.is_playing:
                break
            if self.viseme_callback is not None:
                self.viseme_callback(v[1])

    def play(self) -> None:
        if self.call_viseme_thread is not None:
            self.call_viseme_thread.join()

        arguments = ['aplay', '--quiet', '--channels', '1', '--format', 'S16_LE', '--rate', '16000', '--file-type', 'raw']
        if self.device is not None:
            arguments.extend(['--device', self.device])
        self.aplay_handle = subprocess.Popen(arguments, stdin=subprocess.PIPE)
        self.call_viseme_thread = threading.Thread(target=self.call_viseme)
        self.cancel_event.clear()
        self.call_viseme_thread.start()
        try:
            self.aplay_handle.stdin.write(self.audio_data)
            self.aplay_handle.stdin.close()
        except BrokenPipeError:
            pass

    @property
    def is_playing(self):
        return self.aplay_handle is not None and self.aplay_handle.poll() is None

    def cancel(self):
        if self.is_playing:
            self.aplay_handle.terminate()
            self.cancel_event.set()

    def wait(self):
        if self.is_playing:
            self.aplay_handle.wait()


class SynthesizeResultCacher:
    def __init__(self, cache_path: str):
        self.cache_path = cache_path

    def clean_broken_cache(self):
        if not os.path.exists(self.cache_path):
            return
        for item in os.listdir(self.cache_path):
            item_path = os.path.join(self.cache_path, item)
            audio_data_path = os.path.join(item_path, 'audio_data.pcm')
            meta_data_path = os.path.join(item_path, 'meta_data.json')
            if not os.path.exists(audio_data_path) or not os.path.exists(meta_data_path):
                shutil.rmtree(item_path)

    def get_item_path(self, text: str):
        hash_text = hashlib.sha256(text.encode('utf-8')).hexdigest()
        return os.path.join(self.cache_path, hash_text)

    def get_data_path(self, text: str):
        item_path = self.get_item_path(text)
        audio_data_path = os.path.join(item_path, 'audio_data.pcm')
        meta_data_path = os.path.join(item_path, 'meta_data.json')
        return audio_data_path, meta_data_path

    def get_cache(self, text: str) -> Optional[Tuple[bytes, Iterable[Tuple[float, int]]]]:
        audio_data_path, meta_data_path = self.get_data_path(text)
        if not os.path.exists(audio_data_path) or not os.path.exists(meta_data_path):
            return None
        with open(audio_data_path, 'rb') as f:
            audio_data = f.read()
        with open(meta_data_path, 'r') as f:
            meta_data = json.load(f)
        meta_data['access_counter'] += 1
        with open(meta_data_path, 'w') as f:
            json.dump(meta_data, f, ensure_ascii=False)
        return audio_data, meta_data['viseme_data']

    def cache(self, text: str, audio_data: bytes, viseme_data: Iterable[Tuple[float, int]]):
        item_path = self.get_item_path(text)
        audio_data_path, viseme_data_path = self.get_data_path(text)
        if os.path.exists(item_path):
            return
        os.makedirs(item_path)
        with open(audio_data_path, 'wb') as f:
            f.write(audio_data)
        with open(viseme_data_path, 'w') as f:
            meta_data = {
                'text': text,
                'viseme_data': viseme_data,
                'access_counter': 0
            }
            json.dump(meta_data, f, ensure_ascii=False)


class AzureSpeechService:
    def __init__(self, key, region, language='zh-CN', voice_name='zh-CN-XiaoxiaoNeural', microphone_device=None, speaker_device=None, voice_style=None):
        # init speech recognizer
        speech_config = speechsdk.SpeechConfig(subscription=key, region=region)
        speech_config.speech_recognition_language = language
        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=microphone_device is None, device_name=microphone_device)
        self._language = language
        self._speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

        # init speech synthesizer
        speech_config = speechsdk.SpeechConfig(subscription=key, region=region)
        speech_config.speech_synthesis_voice_name = voice_name
        speech_config.set_speech_synthesis_output_format(speechsdk.SpeechSynthesisOutputFormat.Raw16Khz16BitMonoPcm)
        self._speaker_device = speaker_device
        self._voice_name = voice_name
        self._voice_style = voice_style
        self._speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=None)
        self._speech_synthesizer.viseme_received.connect(self._speech_synthesizer_viseme_received)
        self._speech_synthesizer_viseme_queue = queue.Queue()
        self._viseme_callback: Optional[Callable[[int], None]] = None
        self._synthesize_result: Optional[SynthesizeResult] = None

        # init keyword model
        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=microphone_device is None, device_name=microphone_device)
        self._keyword_recognizer = speechsdk.KeywordRecognizer(audio_config=audio_config)

        # init cacher
        self._synthesize_result_cacher = SynthesizeResultCacher(os.path.join(os.path.dirname(__file__), 'cache'))
        self._synthesize_result_cacher.clean_broken_cache()

    def speech_to_text(self):
        result = self._speech_recognizer.recognize_once_async().get()
        return result.text

    def cancel_speech_to_text(self):
        self._speech_recognizer.stop_continuous_recognition()

    def _speech_synthesizer_viseme_received(self, evt: speechsdk.SpeechSynthesisVisemeEventArgs):
        self._speech_synthesizer_viseme_queue.put((evt.audio_offset / 10000000, evt.viseme_id))

    def set_viseme_callback(self, callback: Optional[Callable[[int], None]]):
        self._viseme_callback = callback

    def text_to_speech(self, text):
        self.wait_speech_synthesising()
        cache = self._synthesize_result_cacher.get_cache(text)
        if cache is None:
            with self._speech_synthesizer_viseme_queue.mutex:
                self._speech_synthesizer_viseme_queue.queue.clear()
            if self._voice_style is None:
                result = self._speech_synthesizer.speak_text_async(text).get()
            else:
                ssml = ssml_template.substitute({
                    "language": self._language,
                    "voice_name": self._voice_name,
                    "voice_style": self._voice_style,
                    "text": text
                })
                result = self._speech_synthesizer.speak_ssml_async(ssml).get()
            audio_data = result.audio_data
            with self._speech_synthesizer_viseme_queue.mutex:
                viseme_data = tuple(self._speech_synthesizer_viseme_queue.queue)
            self._synthesize_result_cacher.cache(text, audio_data, viseme_data)
        else:
            audio_data, viseme_data = cache
        self._synthesize_result = SynthesizeResult(self._speaker_device, audio_data, viseme_data, self._viseme_callback)
        self._synthesize_result.play()

    def stop_speaking(self):
        self._speech_synthesizer.stop_speaking_async().get()
        if self._synthesize_result is not None:
            self._synthesize_result.cancel()

    def is_speech_synthesising(self):
        if self._synthesize_result is None:
            return False
        return self._synthesize_result.is_playing

    def wait_speech_synthesising(self):
        if self._synthesize_result is not None and self._synthesize_result.is_playing:
            self._synthesize_result.wait()

    def recognize_keyword(self, model):
        result = self._keyword_recognizer.recognize_once_async(
            speechsdk.KeywordRecognitionModel(model)
        ).get()
        if result.reason == speechsdk.ResultReason.RecognizedKeyword:
            return True
        return False

    def stop_keyword_recognition(self):
        self._keyword_recognizer.stop_recognition_async().get()
