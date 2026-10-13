"""Event-level execution guard. It does not control real motors."""
from collections import deque
from .planner import plan


class LabelGate:
    def __init__(self,frames=3,min_confidence=.8,min_margin=.15):
        if frames<1 or not 0<=min_confidence<=1 or not 0<=min_margin<=1:raise ValueError("Invalid label gate")
        self.frames=frames;self.min_confidence=min_confidence;self.min_margin=min_margin
        self.history=deque(maxlen=frames)

    def observe(self,label,confidence,runner_up):
        if not 0<=runner_up<=confidence<=1:raise ValueError("Invalid recognition scores")
        if confidence<self.min_confidence or confidence-runner_up<self.min_margin:
            self.history.clear();return None
        self.history.append(label)
        return label if len(self.history)==self.frames and len(set(self.history))==1 else None


class Mission:
    def __init__(self,graph,node,heading,goal,calibration,policy="fastest"):
        self.graph,self.node,self.heading=graph,node,heading
        self.goal,self.calibration,self.policy=goal,calibration,policy
        self.state="IDLE";self.route=None;self.cursor=0;self.pose_confirmed=True
        self.log=[]

    def record(self,event,**fields):
        self.log.append({"event":event,"state":self.state,"node":self.node,"heading":self.heading,
                         "revision":self.graph.revision,**fields})

    def prepare(self):
        if not self.pose_confirmed:raise RuntimeError("Confirm junction pose before replanning")
        self.route=plan(self.graph,self.node,self.goal,self.heading,self.policy,self.calibration)
        self.cursor=0
        self.state="READY" if self.route is not None else "HOLD"
        if self.route is not None and not self.route.steps:self.state="ARRIVED"
        self.record("PLAN",reachable=self.route is not None)
        return self.route

    def next_step(self):
        if self.state!="READY" or not self.route:return None
        if self.route.revision!=self.graph.revision:
            self.state="HOLD";self.record("STALE_PLAN");return None
        step=self.route.steps[self.cursor]
        self.state="MOVING";self.pose_confirmed=False
        self.record("COMMAND",action=step.action,destination=step.dst)
        return step

    def confirm_arrival(self,node):
        if self.state!="MOVING" or node!=self.route.steps[self.cursor].dst:
            self.state="HOLD";self.pose_confirmed=False;self.record("POSE_MISMATCH");return False
        step=self.route.steps[self.cursor]
        self.node,self.heading=node,step.heading
        self.pose_confirmed=True;self.cursor+=1
        self.state="ARRIVED" if node==self.goal else "READY"
        self.record("ARRIVAL");return True

    def obstacle(self,src,dst):
        # A mid-edge stop does not teleport the vehicle back to the last junction.
        self.graph=self.graph.with_block(src,dst)
        self.state="HOLD";self.record("OBSTACLE",src=src,dst=dst)
        if self.pose_confirmed:return self.prepare()
        self.record("NEEDS_LOCALIZATION");return None

    def confirm_pose(self,node,heading):
        if node not in self.graph.index or heading not in range(4):raise ValueError("Invalid pose")
        self.node,self.heading,self.pose_confirmed=node,heading,True
        self.record("POSE_CONFIRMED")
        return self.prepare()
