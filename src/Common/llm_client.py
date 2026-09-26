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

from PyQt5.QtCore import QObject, QUrl, pyqtSignal
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

    HEALTH_TIMEOUT_MS = 5000
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

    def busy(self):
        return self._chat_reply is not None

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
