import argparse
import json
from pathlib import Path
from dataclasses import asdict
from .graph import load_graph,HEADINGS
from .planner import plan,Calibration


def main():
    p=argparse.ArgumentParser(description="Turn-aware routing on a competition practice map")
    p.add_argument("map",type=Path)
    p.add_argument("--start",default="S");p.add_argument("--goal",default="G")
    p.add_argument("--heading",choices=HEADINGS,default="E")
    p.add_argument("--policy",choices=["distance","fastest","robust"],default="fastest")
    p.add_argument("--speed",type=int)
    p.add_argument("--risk-penalty",type=int)
    p.add_argument("--calibration",type=Path)
    p.add_argument("--block",nargs=2,action="append",default=[])
    p.add_argument("--output",type=Path,default=Path("outputs/route.json"))
    args=p.parse_args()
    graph=load_graph(args.map)
    for a,b in args.block:graph=graph.with_block(a,b)
    values=asdict(Calibration())
    if args.calibration:values.update(json.loads(args.calibration.read_text(encoding="utf-8")))
    if args.speed is not None:values["speed_mm_s"]=args.speed
    if args.risk_penalty is not None:values["risk_penalty_ms"]=args.risk_penalty
    c=Calibration(**values)
    route=plan(graph,args.start,args.goal,HEADINGS.index(args.heading),args.policy,c)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    data={"graph":graph.to_dict(),"calibration":asdict(c),"route":route.to_dict() if route else None,
          "status":"reachable" if route else "unreachable"}
    args.output.write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding="utf-8")
    if not route:print("No feasible route. Vehicle should hold.");return 2
    print(" -> ".join([route.start,*[s.dst for s in route.steps]]))
    print(f"{route.distance_mm} mm, {route.turn_count} turns, nominal {route.nominal_time_ms} ms. {args.output}")
    return 0


if __name__=="__main__":raise SystemExit(main())
