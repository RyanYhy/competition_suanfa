"""Print patrol messages and save the same readable text; summary stays last."""
from pathlib import Path
import threading
import time

LOG_NAME='终端日志.log'


class TerminalLog:
    def __init__(self,root):
        self.root=Path(root).expanduser();self.lock=threading.RLock()
        self.active=False;self.finished=False;self.prelude=[];self.last_notice={}

    def prepare(self):
        with self.lock:self.active=False;self.finished=False;self.last_notice={}

    def begin(self,total):
        with self.lock:
            self.root.mkdir(parents=True,exist_ok=True)
            (self.root/LOG_NAME).write_text(''.join(self.prelude),encoding='utf-8')
            self.prelude=[];self.active=True;self.finished=False;self.last_notice={}
            self.emit('[巡航开始] 共%d个航点；结果目录：%s'%(total,self.root))

    def emit(self,message,color=None):
        with self.lock:
            text='[%s] %s\n'%(time.strftime('%H:%M:%S'),message)
            print(('\033[%sm'%color if color else '')+text.rstrip()+('\033[0m' if color else ''),flush=True)
            if self.active and not self.finished:self._append(text)
            elif not self.finished:self.prelude.append(text)

    def _append(self,text):
        with (self.root/LOG_NAME).open('a',encoding='utf-8') as stream:stream.write(text)

    def warning(self,node,message):
        # ROS already prints WARN/ERROR to stderr. Record it without printing twice.
        with self.lock:
            if self.active and not self.finished:
                self._append('[%s] [异常 %s] %s\n'%(time.strftime('%H:%M:%S'),node,message))

    def notice(self,key,message,period=5.):
        with self.lock:
            now=time.monotonic()
            if now-self.last_notice.get(key,float('-inf'))>=period:
                self.last_notice[key]=now;self.emit(message)

    def finish(self,lines,complete=False):
        with self.lock:
            if self.finished:return
            self.emit('\n'.join(lines),color='33' if complete else '31')
            self.finished=True;self.active=False
