'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
"""Qt-native client for the LLM proxy on the NetBird network.

Everything here runs on the Qt GUI thread without blocking it: QNetworkAccessManager
is asynchronous and reports through signals. Nothing in this module touches rclpy.
"""
import json
import os

from PyQt5.QtCore import QObject, QTimer, QUrl, pyqtSignal
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest


class LlmClient(QObject):
    """Health checks and streamed chat against the LLM proxy (Ollama-style API).

    Proxy behaviour observed 2026-09-26: it forwards only `model` and `messages`. It
    always streams, one JSON object per line in Ollama's /api/chat format, whatever
    `stream` says, ignores `options`, rejects an empty `messages` list, and reports
    errors as {"error": "..."}.
    """

    health_checked = pyqtSignal(bool, str, dict)  # ok, error text, parsed /api/health
    chat_delta = pyqtSignal(str)                   # next chunk of the answer
    chat_finished = pyqtSignal(bool, str)          # ok, error text ("" on success)
    parse_finished = pyqtSignal(bool, str, dict)   # ok, error text, parsed /api/parse reply

    HEALTH_TIMEOUT_MS = 5000
    # /api/parse answers in one piece (~0.5 s warm); the bound covers a model reload.
    PARSE_TIMEOUT_MS = 30000
    # Inactivity timeout: Qt restarts it whenever bytes arrive, so it bounds a stall,
    # not the length of an answer. The first token takes 5-10 s when the model has to
    # reload (it unloads after 5 min idle).
    CHAT_IDLE_TIMEOUT_MS = 45000

    def __init__(self, base_url, model, parent=None):
        super().__init__(parent)
        self.base_url = base_url.rstrip('/')
        self.model = model
        self._nam = QNetworkAccessManager(self)
        self._health_reply = None
        self._chat_reply = None
        self._chat_buffer = b''
        self._chat_error = ''
        self._chat_done = False
        self._parse_reply = None
        self._parse_error = ''

    def busy(self):
        return self._chat_reply is not None

    def parse_busy(self):
        return self._parse_reply is not None

    # -- health ------------------------------------------------------------------

    def check_health(self):
        if self._health_reply is not None:
            return  # one check at a time; the answer is on its way
        request = QNetworkRequest(QUrl(self.base_url + '/api/health'))
        request.setTransferTimeout(self.HEALTH_TIMEOUT_MS)
        reply = self._nam.get(request)
        self._health_reply = reply
        reply.finished.connect(lambda: self._on_health_finished(reply))

    def _on_health_finished(self, reply):
        self._health_reply = None
        reply.deleteLater()
        if reply.error() != QNetworkReply.NoError:
            self.health_checked.emit(False, self._describe_error(reply), {})
            return
        try:
            health = json.loads(bytes(reply.readAll()))
        except ValueError:
            self.health_checked.emit(False, 'unreadable health reply', {})
            return
        if not isinstance(health, dict):
            self.health_checked.emit(False, 'unreadable health reply', {})
        elif not health.get('up'):
            self.health_checked.emit(False, 'server reports it is down', health)
        else:
            self.health_checked.emit(True, '', health)

    # -- parse -------------------------------------------------------------------

    def parse(self, utterance):
        """Start a /api/parse request (sentence -> drone command); False if one is
        already running. The server runs nl_parser (aerial_robtoics+vlm): the model
        picks a direction label and an unsigned amount, the sign table is code. The
        reply only says what the sentence means -- resolving it against the vehicle's
        pose and every safety check stay with the caller.
        """
        if self._parse_reply is not None:
            return False
        request = QNetworkRequest(QUrl(self.base_url + '/api/parse'))
        request.setHeader(QNetworkRequest.ContentTypeHeader, 'application/json')
        request.setTransferTimeout(self.PARSE_TIMEOUT_MS)
        body = json.dumps({'utterance': utterance, 'model': self.model,
                           'schema': 'enum', 'constrain': True})
        reply = self._nam.post(request, body.encode('utf-8'))
        self._parse_reply = reply
        self._parse_error = ''
        reply.finished.connect(lambda: self._on_parse_finished(reply))
        return True

    def abort_parse(self, reason='stopped'):
        reply = self._parse_reply
        if reply is None:
            return
        self._parse_error = reason
        reply.abort()
        if self._parse_reply is reply:
            self._on_parse_finished(reply)

    def _on_parse_finished(self, reply):
        reply.deleteLater()
        if reply is not self._parse_reply:
            return
        self._parse_reply = None
        if self._parse_error:
            self.parse_finished.emit(False, self._parse_error, {})
            return
        # The server answers errors with a JSON body too (HTTP 502 for a parser that
        # could not run), so read the body before looking at the HTTP status.
        result = None
        if reply.isOpen():
            try:
                result = json.loads(bytes(reply.readAll()))
            except ValueError:
                result = None
        if isinstance(result, dict) and result.get('outcome') in ('command', 'refusal'):
            self.parse_finished.emit(True, '', result)
        elif isinstance(result, dict) and (result.get('reason') or result.get('error')):
            self.parse_finished.emit(False, str(result.get('reason') or result.get('error')), result)
        elif reply.error() != QNetworkReply.NoError:
            self.parse_finished.emit(False, self._describe_error(reply), {})
        else:
            self.parse_finished.emit(False, 'unreadable reply from /api/parse', {})

    # -- chat --------------------------------------------------------------------

    def chat(self, messages):
        """Start a streamed chat; False if one is already running."""
        if self._chat_reply is not None:
            return False
        request = QNetworkRequest(QUrl(self.base_url + '/api/chat'))
        request.setHeader(QNetworkRequest.ContentTypeHeader, 'application/json')
        request.setTransferTimeout(self.CHAT_IDLE_TIMEOUT_MS)
        body = json.dumps({'model': self.model, 'messages': messages, 'stream': True})
        reply = self._nam.post(request, body.encode('utf-8'))
        self._chat_reply = reply
        self._chat_buffer = b''
        self._chat_error = ''
        self._chat_done = False
        reply.readyRead.connect(lambda: self._on_chat_data(reply))
        reply.finished.connect(lambda: self._on_chat_finished(reply))
        return True

    def abort_chat(self, reason='stopped'):
        reply = self._chat_reply
        if reply is None:
            return
        self._chat_error = reason
        reply.abort()  # normally delivers `finished` synchronously
        if self._chat_reply is reply:
            # Not delivered yet: finish now, so a new chat can start straight away. The
            # late `finished` is then ignored (the reply is no longer current).
            self._on_chat_finished(reply)

    def _on_chat_data(self, reply):
        if reply is not self._chat_reply or not reply.isOpen():
            return  # an aborted reply is closed; reading it only makes Qt warn
        self._chat_buffer += bytes(reply.readAll())
        *lines, self._chat_buffer = self._chat_buffer.split(b'\n')
        for line in lines:
            self._handle_line(line)

    def _handle_line(self, line):
        line = line.strip()
        if not line:
            return
        try:
            item = json.loads(line)
        except ValueError:
            self._chat_error = self._chat_error or 'unreadable reply from the LLM server'
            return
        if not isinstance(item, dict):
            return
        if 'error' in item:
            self._chat_error = self._chat_error or str(item['error'])
            return
        content = (item.get('message') or {}).get('content')
        if content:
            self.chat_delta.emit(content)
        if item.get('done'):
            self._chat_done = True

    def _on_chat_finished(self, reply):
        reply.deleteLater()
        if reply is not self._chat_reply:
            return
        self._on_chat_data(reply)
        if self._chat_buffer:  # a last line without a trailing newline
            self._handle_line(self._chat_buffer)
            self._chat_buffer = b''
        # Prefer the server's own message (an HTTP 400 carries {"error": ...}) and a
        # user stop over Qt's generic error string.
        error = self._chat_error
        if not error and reply.error() != QNetworkReply.NoError:
            error = self._describe_error(reply)
        if not error and not self._chat_done:
            error = 'the reply ended early'
        self._chat_reply = None
        self.chat_finished.emit(not error, error)

    @staticmethod
    def _describe_error(reply):
        if reply.error() == QNetworkReply.OperationCanceledError:
            # Our own aborts set their reason first, so a bare cancel is Qt's
            # transfer timeout firing.
            return 'timed out'
        status = reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)
        if status:
            return f'HTTP {status}'
        return reply.errorString()


OPENROUTER_KEY_FILE = os.path.expanduser('~/.config/fsc_gs/openrouter_key')


def openrouter_key():
    """The OpenRouter API key: $OPENROUTER_API_KEY, else the first line of
    ~/.config/fsc_gs/openrouter_key. Never kept in the repo or the machine config."""
    key = os.environ.get('OPENROUTER_API_KEY', '').strip()
    if key:
        return key
    try:
        with open(OPENROUTER_KEY_FILE) as fh:
            return fh.readline().strip()
    except OSError:
        return ''


class OpenRouterClient(QObject):
    """A cloud VLM for camera checks, with LlmClient's chat interface (chat, busy,
    abort_chat, chat_delta, chat_finished) so the camera search can use either.

    OpenRouter's OpenAI-compatible /chat/completions, not streamed (a verdict is two short
    lines). Messages come in Ollama form -- an 'images' list of base64 JPEGs -- and are
    converted to OpenAI content parts here. temperature 0: the same frame should get the
    same verdict. Every call sends the camera frame off-site.
    """

    chat_delta = pyqtSignal(str)
    chat_finished = pyqtSignal(bool, str)
    TIMEOUT_MS = 30000
    BASE_URL = 'https://openrouter.ai/api/v1'

    def __init__(self, model, base_url=None, parent=None):
        super().__init__(parent)
        self.model = model
        self.base_url = (base_url or self.BASE_URL).rstrip('/')
        self._nam = QNetworkAccessManager(self)
        self._reply = None
        self._error = ''

    def busy(self):
        return self._reply is not None

    @staticmethod
    def _convert(messages):
        out = []
        for m in messages:
            images = m.get('images') or []
            if not images:
                out.append({'role': m['role'], 'content': m['content']})
                continue
            parts = [{'type': 'text', 'text': m['content']}]
            parts += [{'type': 'image_url',
                       'image_url': {'url': 'data:image/jpeg;base64,' + img}} for img in images]
            out.append({'role': m['role'], 'content': parts})
        return out

    def chat(self, messages):
        if self._reply is not None:
            return False
        key = openrouter_key()
        if not key:
            # Report through the normal finish path, after the caller has set its state.
            QTimer.singleShot(0, lambda: self.chat_finished.emit(
                False, 'no OpenRouter API key (set OPENROUTER_API_KEY or put it in '
                       f'{OPENROUTER_KEY_FILE})'))
            return True
        request = QNetworkRequest(QUrl(self.base_url + '/chat/completions'))
        request.setHeader(QNetworkRequest.ContentTypeHeader, 'application/json')
        request.setRawHeader(b'Authorization', f'Bearer {key}'.encode())
        request.setRawHeader(b'X-Title', b'FSC ground station')
        request.setTransferTimeout(self.TIMEOUT_MS)
        body = json.dumps({'model': self.model, 'messages': self._convert(messages),
                           'temperature': 0, 'max_tokens': 300})
        reply = self._nam.post(request, body.encode('utf-8'))
        self._reply = reply
        self._error = ''
        reply.finished.connect(lambda: self._on_finished(reply))
        return True

    def abort_chat(self, reason='stopped'):
        reply = self._reply
        if reply is None:
            return
        self._error = reason
        reply.abort()
        if self._reply is reply:
            self._on_finished(reply)

    def _on_finished(self, reply):
        reply.deleteLater()
        if reply is not self._reply:
            return
        self._reply = None
        if self._error:
            self.chat_finished.emit(False, self._error)
            return
        result = None
        if reply.isOpen():
            try:
                result = json.loads(bytes(reply.readAll()))
            except ValueError:
                result = None
        if isinstance(result, dict) and result.get('error'):
            err = result['error']
            self.chat_finished.emit(False, 'OpenRouter: ' + str(
                err.get('message') if isinstance(err, dict) else err))
            return
        if reply.error() != QNetworkReply.NoError:
            self.chat_finished.emit(False, LlmClient._describe_error(reply))
            return
        try:
            text = result['choices'][0]['message']['content'] or ''
        except (TypeError, KeyError, IndexError):
            self.chat_finished.emit(False, 'unreadable reply from OpenRouter')
            return
        if isinstance(text, list):  # content parts
            text = ''.join(p.get('text', '') for p in text if isinstance(p, dict))
        self.chat_delta.emit(text)
        self.chat_finished.emit(True, '')
