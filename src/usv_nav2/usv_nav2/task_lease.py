"""Optional fail-closed Task1 permission; independent of ROS for fault tests."""
from .core import stamp_is_fresh


class TaskLease:
    def __init__(self, timeout=.5):
        self.timeout=timeout;self.revision=-1;self.stamp=0.;self.received=None
        self.allow=False;self.reason='task permission missing'

    def update(self, revision, allow, stamp, sim_now, wall_now, reason=''):
        if revision<self.revision or stamp<=self.stamp or not stamp_is_fresh(stamp,sim_now,self.timeout):
            return False
        changed=revision!=self.revision or allow!=self.allow or not self.valid(sim_now,wall_now)
        self.revision=revision;self.allow=allow;self.stamp=stamp;self.received=wall_now;self.reason=reason
        return changed

    def valid(self, sim_now, wall_now):
        return (self.allow and self.received is not None and 0<=wall_now-self.received<=self.timeout
                and stamp_is_fresh(self.stamp,sim_now,self.timeout))
