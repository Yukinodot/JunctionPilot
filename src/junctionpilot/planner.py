from dataclasses import dataclass, asdict
import heapq
import math
from .graph import HEADINGS


@dataclass(frozen=True)
class Calibration:
    speed_mm_s: int = 300
    left_ms: int = 850
    right_ms: int = 650
    uturn_ms: int = 1600
    settle_ms: int = 100
    vehicle_width_mm: int = 150
    clearance_mm: int = 20
    risk_penalty_ms: int = 8000

    def __post_init__(self):
        values=asdict(self)
        if not all(isinstance(v,int) and not isinstance(v,bool) for v in values.values()):
            raise ValueError("Calibration values must be integers")
        if not 50 <= self.speed_mm_s <= 5000 or not 1 <= self.vehicle_width_mm <= 10000:
            raise ValueError("Invalid speed/vehicle width")
        if not 0 <= self.clearance_mm <= 5000 or any(not 0 <= values[k] <= 60000 for k in
              ("left_ms","right_ms","uturn_ms","settle_ms","risk_penalty_ms")):
            raise ValueError("Calibration outside portable C bounds")


@dataclass(frozen=True)
class Step:
    src: str
    dst: str
    heading: int
    action: str
    travel_ms: int
    turn_ms: int
    risk_cost_ms: int
    length_mm: int
    risk_permille: int


@dataclass(frozen=True)
class Route:
    start: str
    goal: str
    start_heading: int
    policy: str
    revision: int
    cost: int
    steps: tuple[Step,...]

    @property
    def nominal_time_ms(self):return sum(s.travel_ms+s.turn_ms for s in self.steps)
    @property
    def distance_mm(self):return sum(s.length_mm for s in self.steps)
    @property
    def turn_count(self):return sum(s.action!="STRAIGHT" for s in self.steps)
    @property
    def risk_proxy(self):return sum(s.risk_permille for s in self.steps)
    @property
    def assumed_failure_probability(self):
        return 1-math.prod(1-s.risk_permille/1000 for s in self.steps)

    def to_dict(self):
        return {**asdict(self),"nominal_time_ms":self.nominal_time_ms,"distance_mm":self.distance_mm,
                "turn_count":self.turn_count,"risk_proxy":self.risk_proxy,
                "assumed_failure_probability":self.assumed_failure_probability,
                "cost_unit":"mm" if self.policy=="distance" else "ms"}


def turn(incoming,outgoing,calibration):
    delta=(outgoing-incoming)%4
    action=("STRAIGHT","RIGHT","UTURN","LEFT")[delta]
    base=(0,calibration.right_ms,calibration.uturn_ms,calibration.left_ms)[delta]
    return action,base+(calibration.settle_ms if delta else 0)


def usable(edge,c):
    return not edge.blocked and edge.width_mm >= c.vehicle_width_mm+2*c.clearance_mm


def edge_step(graph,edge,incoming,c):
    heading=graph.heading(edge)
    action,turn_ms=turn(incoming,heading,c)
    travel=(edge.length_mm*1000+c.speed_mm_s-1)//c.speed_mm_s
    risk=(edge.risk_permille*c.risk_penalty_ms+999)//1000
    return Step(edge.src,edge.dst,heading,action,travel,turn_ms,risk,edge.length_mm,edge.risk_permille)


def step_cost(step,policy):
    if policy=="distance":return step.length_mm
    return step.travel_ms+step.turn_ms+(step.risk_cost_ms if policy=="robust" else 0)


def plan(graph,start,goal,heading=1,policy="fastest",calibration=Calibration()):
    """Exact Dijkstra on (junction, heading), with nonnegative integer costs."""
    if start not in graph.index or goal not in graph.index or heading not in range(4):
        raise ValueError("Invalid start, goal, or heading")
    if policy not in {"distance","fastest","robust"}:raise ValueError("Unknown policy")
    initial=4*graph.index[start]+heading
    distance={initial:0}
    parent={}
    queue=[(0,initial)]
    settled=set()
    while queue:
        cost,state=heapq.heappop(queue)
        if state in settled:continue
        settled.add(state)
        node=graph.nodes[state//4].id
        if node==goal:
            steps=[]
            end=state
            while end!=initial:
                prev,step=parent[end];steps.append(step);end=prev
            return Route(start,goal,heading,policy,graph.revision,cost,tuple(reversed(steps)))
        for _,edge in graph.adj[node]:
            if not usable(edge,calibration):continue
            step=edge_step(graph,edge,state%4,calibration)
            nxt=4*graph.index[edge.dst]+step.heading
            candidate=cost+step_cost(step,policy)
            if candidate < distance.get(nxt,1<<62):
                distance[nxt]=candidate;parent[nxt]=(state,step)
                heapq.heappush(queue,(candidate,nxt))
    return None
