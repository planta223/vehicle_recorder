"""Optional controlling-terminal UI, also usable by a ros2 launch child."""
import os
import select
import termios
import tty


class RecordTerminal:
    def __init__(self, enabled=True):
        self.fd = None
        self.original = None
        if enabled:
            try:
                # launch pipes stdin/stdout; /dev/tty still refers to the operator's terminal.
                self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
                self.original = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd, termios.TCSANOW)
            except (OSError, termios.error):
                self.close()

    def write(self, text):
        os.write(self.fd, text.encode('utf-8'))

    def quit_requested(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return False
        return os.read(self.fd, 1).lower() == b'q'

    def restore(self):
        if self.fd is not None and self.original is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.original)

    def readline(self, prompt, stopping):
        self.write(prompt)
        while not stopping.is_set():
            if select.select([self.fd], [], [], .1)[0]:
                data = os.read(self.fd, 4096)
                return data.decode('utf-8').strip() if data else None
        return None

    def review(self, recorder, stopping):
        self.restore()
        self.write('\n기록 종료: 저장된 데이터를 확인합니다.\n')
        while True:
            answer = self.readline('정상적인 기록입니까? y/n: ', stopping)
            if answer is None:
                return
            if answer.lower() in ('y', 'n'):
                break
            self.write('y 또는 n을 입력해주세요.\n')
        name = None
        if answer.lower() == 'y':
            while True:
                name = self.readline('기록 이름을 입력해주세요. ', stopping)
                if name is None:
                    return
                try:
                    recorder.validate_name(name)
                    break
                except ValueError as error:
                    self.write(str(error) + '\n')
        recorder.review(answer.lower() == 'y', name)
        self.write(('정상 기록 저장: ' if name else '비정상 기록 보존: ') + str(recorder.directory) + '\n')

    def close(self):
        if self.fd is not None:
            try:
                self.restore()
            finally:
                os.close(self.fd)
                self.fd = None
