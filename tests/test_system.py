from pathlib import Path
import unittest
from junctionpilot.graph import Graph,Node,Edge,load_graph
from junctionpilot.planner import plan,Calibration,turn
from junctionpilot.mission import Mission,LabelGate

ROOT=Path(__file__).resolve().parents[1]


class PlannerTests(unittest.TestCase):
    def setUp(self):self.graph=load_graph(ROOT/"maps/turn_tradeoff.json");self.c=Calibration()

    def test_longer_route_can_save_turn_time(self):
        a=plan(self.graph,"S","G",1,"distance",self.c)
        b=plan(self.graph,"S","G",1,"fastest",self.c)
        self.assertGreater(b.distance_mm,a.distance_mm);self.assertLess(b.nominal_time_ms,a.nominal_time_ms)

    def test_heading_changes_cost(self):
        a=plan(self.graph,"S","U",0,"fastest",self.c)
        b=plan(self.graph,"S","U",1,"fastest",self.c)
        self.assertGreater(b.cost,a.cost)

    def test_start_equals_goal(self):
        r=plan(self.graph,"S","S");self.assertEqual(r.cost,0);self.assertEqual(r.steps,())

    def test_no_route(self):
        g=self.graph.with_block("S","A").with_block("S","U")
        self.assertIsNone(plan(g,"S","G"))

    def test_block_revises_graph_without_mutation(self):
        g=self.graph.with_block("S","U")
        self.assertEqual(g.revision,1);self.assertEqual(self.graph.revision,0)
        self.assertTrue(all(not (s.src=="S" and s.dst=="U") for s in plan(g,"S","G").steps))

    def test_vehicle_clearance_filters_edges(self):
        c=Calibration(vehicle_width_mm=400)
        self.assertIsNone(plan(self.graph,"S","G",calibration=c))

    def test_clearance_inclusive_boundary(self):
        g=Graph([Node("a",0,0),Node("b",1,0)],[Edge("a","b",200,190)])
        self.assertIsNotNone(plan(g,"a","b"))

    def test_left_right_uturn_and_straight(self):
        self.assertEqual(turn(1,0,self.c),("LEFT",950))
        self.assertEqual(turn(0,1,self.c),("RIGHT",750))
        self.assertEqual(turn(0,2,self.c),("UTURN",1700))
        self.assertEqual(turn(3,3,self.c),("STRAIGHT",0))

    def test_risk_detour(self):
        g=load_graph(ROOT/"maps/risk_detour.json")
        a=plan(g,"S","G",policy="fastest");b=plan(g,"S","G",policy="robust")
        self.assertLess(b.risk_proxy,a.risk_proxy);self.assertGreater(b.nominal_time_ms,a.nominal_time_ms)

    def test_zero_risk_weight_agrees_with_fastest(self):
        a=plan(self.graph,"S","G",policy="fastest",calibration=Calibration(risk_penalty_ms=0))
        b=plan(self.graph,"S","G",policy="robust",calibration=Calibration(risk_penalty_ms=0))
        self.assertEqual(a.steps,b.steps);self.assertEqual(a.cost,b.cost)

    def test_roundtrip_uses_actual_final_heading(self):
        r=plan(self.graph,"S","G");back=plan(self.graph,"G","S",r.steps[-1].heading)
        self.assertEqual(back.start_heading,r.steps[-1].heading);self.assertEqual(back.steps[-1].dst,"S")

    def test_invalid_graphs(self):
        with self.assertRaises(ValueError):Graph([Node("a",0,0),Node("b",1,1)],[Edge("a","b",100)])
        with self.assertRaises(ValueError):Graph([Node("a",0,0),Node("a",1,0)],[])
        with self.assertRaises(ValueError):Graph([Node("a",0,0),Node("b",1,0)],[Edge("a","b",0)])
        with self.assertRaises(ValueError):Graph([Node("a",0,0),Node("b",1,0)],[Edge("a","b",100,risk_permille=1001)])

    def test_invalid_inputs(self):
        for kwargs in [{"heading":4},{"policy":"magic"},{"start":"unknown"}]:
            with self.assertRaises(ValueError):plan(self.graph,goal="G",**({"start":"S"}|kwargs))
        with self.assertRaises(ValueError):Calibration(speed_mm_s=0)
        with self.assertRaises(ValueError):Calibration(left_ms=-1)

    def test_exact_state_graph_oracle(self):
        # Bellman-Ford relaxations are independent of the planner's heap search.
        from junctionpilot.planner import edge_step,step_cost,usable
        from scripts.benchmark import random_graph
        for seed in range(20):
            g=random_graph(seed,size=3)
            for policy in ["distance","fastest","robust"]:
                costs=[float("inf")]*(len(g.nodes)*4);costs[1]=0
                for _ in range(len(costs)-1):
                    changed=False
                    for state in range(len(costs)):
                        node=g.nodes[state//4].id
                        for _,edge in g.adj[node]:
                            if not usable(edge,self.c):continue
                            step=edge_step(g,edge,state%4,self.c);nxt=g.index[edge.dst]*4+step.heading
                            candidate=costs[state]+step_cost(step,policy)
                            if candidate<costs[nxt]:costs[nxt]=candidate;changed=True
                    if not changed:break
                expected=min(costs[8*4:8*4+4]);actual=plan(g,"0","8",1,policy,self.c)
                self.assertEqual(actual.cost if actual else float("inf"),expected)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.graph=load_graph(ROOT/"maps/turn_tradeoff.json")
        self.m=Mission(self.graph,"S",1,"G",Calibration());self.m.prepare()

    def test_normal_mission_arrives(self):
        while self.m.state!="ARRIVED":
            step=self.m.next_step();self.assertIsNotNone(step);self.assertTrue(self.m.confirm_arrival(step.dst))

    def test_mid_edge_obstacle_requires_pose(self):
        step=self.m.next_step();self.assertIsNone(self.m.obstacle(step.src,step.dst))
        self.assertEqual(self.m.state,"HOLD");self.assertFalse(self.m.pose_confirmed)
        with self.assertRaises(RuntimeError):self.m.prepare()

    def test_junction_obstacle_replans(self):
        step=self.m.route.steps[0]
        r=self.m.obstacle(step.src,step.dst)
        self.assertIsNotNone(r);self.assertEqual(r.revision,1);self.assertEqual(self.m.state,"READY")

    def test_wrong_arrival_holds(self):
        self.m.next_step();self.assertFalse(self.m.confirm_arrival("S"));self.assertEqual(self.m.state,"HOLD")

    def test_stale_plan_is_rejected(self):
        self.m.graph=self.graph.with_block("S","U")
        self.assertIsNone(self.m.next_step());self.assertEqual(self.m.state,"HOLD")

    def test_relocalize_and_replan(self):
        self.m.next_step();self.m.obstacle("S","U")
        route=self.m.confirm_pose("S",1);self.assertIsNotNone(route);self.assertEqual(self.m.state,"READY")


class PerceptionGateTests(unittest.TestCase):
    def test_three_consistent_frames(self):
        g=LabelGate();self.assertIsNone(g.observe(5,.95,.05));self.assertIsNone(g.observe(5,.95,.05));self.assertEqual(g.observe(5,.95,.05),5)
    def test_low_confidence_clears_history(self):
        g=LabelGate();g.observe(5,.95,.05);g.observe(5,.95,.05)
        self.assertIsNone(g.observe(5,.6,.1));self.assertIsNone(g.observe(5,.95,.05))
    def test_ambiguous_margin_clears_history(self):
        g=LabelGate();self.assertIsNone(g.observe(5,.85,.8))
    def test_disagreement_does_not_confirm(self):
        g=LabelGate();g.observe(5,.95,.05);g.observe(4,.95,.05);self.assertIsNone(g.observe(5,.95,.05))


if __name__=="__main__":unittest.main()
