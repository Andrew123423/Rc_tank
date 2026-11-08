"""GUI-independent held-key state; deterministic and testable without ROS/Tk."""
import math

class KeyState:
    aliases={'ц':'w','ы':'s','ф':'a','в':'d','й':'q','у':'e','я':'z','с':'c',
             'Cyrillic_tse':'w','Cyrillic_yeru':'s','Cyrillic_ef':'a','Cyrillic_ve':'d'}
    def __init__(self,speed=.35):
        if not math.isfinite(speed):raise ValueError('speed must be finite')
        self.speed=max(.05,min(1.,speed));self.held=set();self.blocked=set();self.gear=1;self.light=0
    def normalise(self,key):return self.aliases.get(key,self.aliases.get(key.lower(),key.lower()))
    def press(self,key):
        key=self.normalise(key);fresh=key not in self.held
        if key in ('space','escape'):
            self.blocked.update(self.held);self.held.clear();return
        if key in self.blocked:return
        self.held.add(key)
        if not fresh:return
        if key in ('1','2'):self.gear=int(key)
        elif key=='f':self.light=(self.light+1)%5
        elif key in ('0','b','n','m','p'):self.light={'0':0,'b':1,'n':2,'m':3,'p':4}[key]
        elif key in ('plus','equal','kp_add','+','='):self.speed=min(1.,self.speed+.05)
        elif key in ('minus','kp_subtract','-'):self.speed=max(.05,self.speed-.05)
    def release(self,key):
        key=self.normalise(key);self.held.discard(key);self.blocked.discard(key)
    def clear(self,block=False):
        if block:self.blocked.update(self.held)
        else:self.blocked.clear()
        self.held.clear()
    def command(self):
        keys=self.held
        forward=int(bool(keys&{'w','up','q','e'}))-int(bool(keys&{'s','down','z','c'}))
        turn=int(bool(keys&{'a','left','q','z'}))-int(bool(keys&{'d','right','e','c'}))
        # Half steering while translating; pure A/D turns at the selected duty.
        return forward*self.speed,turn*self.speed*(.5 if forward else 1.)
