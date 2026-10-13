"""Deterministic synthetic routing benchmark; all time/risk assumptions are explicit."""
import csv
from dataclasses import asdict
import json
import gzip
from pathlib import Path
import platform
import random
import statistics
import time
from junctionpilot.graph import Graph,Node,Edge
from junctionpilot.planner import Calibration,plan


def random_graph(seed,size=5):
    rng=random.Random(seed)
    nodes=[Node(str(y*size+x),x,y) for y in range(size) for x in range(size)]
    edges=[]
    for n in nodes:
        for dx,dy in [(1,0),(0,1)]:
            x,y=n.x+dx,n.y+dy
            if x>=size or y>=size or rng.random()<.17:continue
            dst=str(y*size+x)
            length=rng.choice([180,200,250,300,400])
            width=rng.choice([170,220,300,350])
            risk=rng.choice([0,5,10,30,100,250])
            blocked=rng.random()<.06
            edges.extend([Edge(n.id,dst,length,width,risk,blocked),Edge(dst,n.id,length,width,risk,blocked)])
    return Graph(nodes,edges,title=f"Synthetic grid seed={seed}")


def main():
    output=Path("reports");output.mkdir(exist_ok=True)
    c=Calibration()
    rows,fixtures=[],[]
    timings={p:[] for p in ("distance","fastest","robust")}
    for seed in range(200):
        g=random_graph(seed)
        routes={}
        heading=seed%4
        for policy in timings:
            samples=[]
            for _ in range(5):
                start=time.perf_counter_ns();r=plan(g,"0","24",heading,policy,c)
                samples.append((time.perf_counter_ns()-start)/1e6)
            timings[policy].extend(samples)
            routes[policy]=r.to_dict() if r else None
            row={"seed":seed,"policy":policy,"reachable":r is not None,"nominal_time_ms":r.nominal_time_ms if r else None,
                 "distance_mm":r.distance_mm if r else None,"turns":r.turn_count if r else None,
                 "risk_proxy":r.risk_proxy if r else None,"assumed_failure_probability":r.assumed_failure_probability if r else None,
                 "cost":r.cost if r else None,"planning_median_ms":statistics.median(samples)}
            rows.append(row)
        fixtures.append({"seed":seed,"graph":g.to_dict(),"start":"0","goal":"24","heading":heading,
                         "calibration":asdict(c),"routes":routes})
    with (output/"per_case.csv").open("w",encoding="utf-8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    payload=json.dumps(fixtures,separators=(",",":")).encode("utf-8")
    (output/"cases.json").write_bytes(payload)
    (output/"cases.json.gz").write_bytes(gzip.compress(payload,compresslevel=9,mtime=0))
    feasible=[f for f in fixtures if f["routes"]["fastest"] is not None]
    assert all((f["routes"]["distance"] is None)==(f["routes"]["fastest"] is None)==(f["routes"]["robust"] is None) for f in fixtures)
    gains=[(f["routes"]["distance"]["nominal_time_ms"]-f["routes"]["fastest"]["nominal_time_ms"])/f["routes"]["distance"]["nominal_time_ms"] for f in feasible]
    assert all(g>=0 for g in gains)
    summary={"maps":len(fixtures),"queries":len(rows),"feasible_maps":len(feasible),"seed_range":[0,199],
             "python":platform.python_version(),"calibration":asdict(c),
             "fastest_vs_distance_mean_nominal_time_reduction":statistics.mean(gains) if gains else None,
             "fastest_vs_distance_improved_maps":sum(g>0 for g in gains),
             "robust_lower_risk_than_fastest_maps":sum(f["routes"]["robust"]["risk_proxy"]<f["routes"]["fastest"]["risk_proxy"] for f in feasible),
             "planning_median_ms":{p:statistics.median(t) for p,t in timings.items()},
             "evidence_scope":"Synthetic graph simulation only; no physical track, perception, or motor run",
             "risk_note":"Hand-assigned edge scores; independent-event probability shown only as a model assumption"}
    (output/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    md=["# 仿真结果", "", "200 张固定种子的自建 5×5 拓扑地图，3 种策略，5 次规划计时重复。道路长度、风险和标定参数均为合成设定，未测量实车。", "",
        f"- 可达地图：{summary['feasible_maps']} / 200。不可达时三种策略都返回无路线。",
        f"- 考虑转弯的最快策略在 {summary['fastest_vs_distance_improved_maps']} 张可达图上缩短了名义时间；相对最短距离策略的平均降低比例为 {100*summary['fastest_vs_distance_mean_nominal_time_reduction']:.2f}%。",
        f"- 风险加权策略在 {summary['robust_lower_risk_than_fastest_maps']} 张可达图上降低了风险分数之和，可能增加行驶时间。", "",
        "这些结果说明代码在给定成本模型下如何选择路线，不证明实车变快，也不证明真实失误概率下降。最快策略按同一名义时间模型优化，因此优于距离基线是模型内的预期性质。", "",
        "原始案例在 cases.json.gz，可运行 scripts/unpack_cases.py 解压；逐策略 CSV 在 per_case.csv。Python/C/JavaScript 一致性验证由独立脚本执行。"]
    (output/"RESULTS.md").write_text("\n".join(md)+"\n",encoding="utf-8")
    print(json.dumps(summary,indent=2))


if __name__=="__main__":main()
