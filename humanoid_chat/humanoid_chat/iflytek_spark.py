import base64
import datetime
import hashlib
import hmac
import json
import threading
import urllib.parse
import websocket
import ssl
import queue
import random
import string


class AuthorizeUrl:
    def __init__(self, api_key, api_secret, host, path):
        self._api_key = api_key
        self._api_secret = api_secret
        self._host = host
        self._path = path

    def __str__(self):
        return self.create_url()

    @staticmethod
    def _timestamp_rfc1123():
        dt = datetime.datetime.utcnow()
        weekday = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"][dt.weekday()]
        month = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep",
                 "Oct", "Nov", "Dec"][dt.month - 1]
        return "%s, %02d %s %04d %02d:%02d:%02d GMT" % (weekday, dt.day, month,
                                                        dt.year, dt.hour, dt.minute, dt.second)

    def create_url(self):
        url_base = f'wss://{self._host}{self._path}'

        date = self._timestamp_rfc1123()
        request_line = f"GET {self._path} HTTP/1.1"
        signature_origin = '\n'.join((f'host: {self._host}', f'date: {date}', request_line))

        signature_sha = hmac.new(self._api_secret.encode('utf-8'), signature_origin.encode('utf-8'), hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode('utf-8')

        authorization_origin = f'api_key="{self._api_key}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')

        v = {
            "authorization": authorization,
            "date": date,
            "host": self._host
        }
        url = url_base + '?' + urllib.parse.urlencode(v)

        return url


class SparkDesk:
    def __init__(self, app_id, api_key, api_secret, temperature=0.5, max_tokens=2048, top_k=4):
        self._url = AuthorizeUrl(api_key, api_secret, 'spark-api.xf-yun.com', '/v3.1/chat')
        self._app_id = app_id
        self._temperature = temperature
        self._max_tokens = max_tokens
        self._top_k = top_k
        self._uid = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(32))
        self._total_tokens = []
        self._chat_history = []
        self._total_response = ""
        self._response_queue = queue.Queue()
        self._chat_thread = None
        self._breaking_flag = False

    def _generate_params(self):
        data = {
            "header": {
                "app_id": self._app_id,
                "uid": self._uid
            },
            "parameter": {
                "chat": {
                    "domain": "generalv3",
                    "temperature": self._temperature,
                    "max_tokens": self._max_tokens,
                    "top_k": self._top_k
                }
            },
            "payload": {
                "message": {
                    "text": self._chat_history
                }
            }
        }
        return data

    def _on_message(self, ws, message):
        if self._breaking_flag:
            ws.close()
            self._response_queue.put(None)
        msg = json.loads(message)
        code = msg["header"]["code"]
        if code != 0:
            raise RuntimeError(f'sid: {msg["header"]["sid"]}, message: {msg["header"]["message"]}')
        self._total_response += msg["payload"]["choices"]["text"][0]["content"]
        self._response_queue.put(self._total_response)
        if msg["payload"]["choices"]["status"] == 2:
            self._total_tokens.append(msg["payload"]["usage"]["text"]["total_tokens"])
            self._response_queue.put(None)
            ws.close()
    
    def break_stream(self):
        self._breaking_flag = True

    @property
    def tokens(self):
        return self._total_tokens

    def _on_error(self, ws, error):
        print(error)

    def _on_close(self, ws, close_status_code, close_msg):
        self._chat_history.append({"role": "assistant", "content": self._total_response})

    def _on_open(self, ws):
        self._total_response = ""
        self._breaking_flag = False
        ws.send(json.dumps(self._generate_params()))

    def reset(self):
        self._total_tokens.clear()
        self._chat_history.clear()

    def chat(self, question):
        self._chat_history.append({"role": "user", "content": question})

        ws = websocket.WebSocketApp(self._url.create_url(),
                                    on_open=self._on_open,
                                    on_message=self._on_message,
                                    on_error=self._on_error,
                                    on_close=self._on_close)
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

        return self._total_response

    def chat_stream(self, question, timeout=10.0):
        if self._chat_thread is not None:
            if self._chat_thread.is_alive():
                self._chat_thread.join()
        if not self._response_queue.empty():
            with self._response_queue.mutex:
                self._response_queue.queue.clear()

        self._chat_thread = threading.Thread(target=self.chat, args=[question])
        self._chat_thread.start()
        while True:
            try:
                response = self._response_queue.get(timeout=timeout)
            except queue.Empty:
                self._chat_thread.join()
                self._chat_history.clear()
                yield "非常抱歉，我还无法回答您的问题。"
                break
            if response is None:
                break
            yield response
