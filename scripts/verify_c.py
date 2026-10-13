"""Compile the actual portable C implementation and compare against saved Python queries."""
import argparse
import ctypes as ct
import json
import gzip
import os
from pathlib import Path
import platform
import shutil
import subprocess

MAX_N,MAX_E,STATES=64,256,256
class CEdge(ct.Structure):
    _fields_=[("src",ct.c_uint8),("dst",ct.c_uint8),("heading",ct.c_uint8),("blocked",ct.c_uint8),
              ("length_mm",ct.c_uint16),("width_mm",ct.c_uint16),("risk_permille",ct.c_uint16)]
class CGraph(ct.Structure):
    _fields_=[("node_count",ct.c_uint16),("edge_count",ct.c_uint16),("revision",ct.c_uint32),("edges",CEdge*MAX_E)]
class CCalibration(ct.Structure):
    _fields_=[(n,ct.c_uint16) for n in ("speed_mm_s","left_ms","right_ms","uturn_ms","settle_ms","vehicle_width_mm","clearance_mm","risk_penalty_ms")]
class CStep(ct.Structure):
    _fields_=[("edge_index",ct.c_uint16),("src",ct.c_uint8),("dst",ct.c_uint8),("heading",ct.c_uint8),("action",ct.c_uint8),
              ("travel_ms",ct.c_uint32),("turn_ms",ct.c_uint32),("risk_cost_ms",ct.c_uint32),("length_mm",ct.c_uint16),("risk_permille",ct.c_uint16)]
class CRoute(ct.Structure):
    _fields_=[("cost",ct.c_uint32),("nominal_time_ms",ct.c_uint32),("distance_mm",ct.c_uint32),("turn_count",ct.c_uint16),("step_count",ct.c_uint16),
              ("revision",ct.c_uint32),("steps",CStep*STATES)]
class CWorkspace(ct.Structure):
    _fields_=[("distance",ct.c_uint32*STATES),("parent",ct.c_int16*STATES),("parent_edge",ct.c_int16*STATES),("settled",ct.c_uint8*STATES)]
class CGuard(ct.Structure):
    _fields_=[("plan_revision",ct.c_uint32),("lost_line_ticks",ct.c_uint16),("pose_confirmed",ct.c_uint8),("latched_hold",ct.c_uint8)]


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--cc",default=shutil.which("gcc") or "gcc")
    args=parser.parse_args()
    build=Path("build");build.mkdir(exist_ok=True)
    suffix=".dll" if os.name=="nt" else ".so"
    library=build/("jp_planner"+suffix)
    command=[args.cc,"-std=c11","-Wall","-Wextra","-Werror","-O2","-shared"]
    if os.name!="nt":command.append("-fPIC")
    command += ["firmware/jp_planner.c","firmware/jp_guard.c","-o",str(library)]
    compile_run=subprocess.run(command,text=True,capture_output=True,check=True)
    log=["COMPILE: "+" ".join(command),compile_run.stdout,compile_run.stderr]
    compiler_version=subprocess.check_output([args.cc,"--version"],text=True).splitlines()[0]
    if os.name=="nt":os.add_dll_directory(str(Path(args.cc).resolve().parent))
    lib=ct.CDLL(str(library.resolve()))
    for name,struct in [("workspace",CWorkspace),("graph",CGraph),("route",CRoute)]:
        fn=getattr(lib,"jp_"+name+"_size");fn.restype=ct.c_size_t
        assert fn()==ct.sizeof(struct),(name,fn(),ct.sizeof(struct))
    lib.jp_plan.argtypes=[ct.POINTER(CGraph),ct.c_uint8,ct.c_uint8,ct.c_uint8,ct.c_uint8,ct.POINTER(CCalibration),ct.POINTER(CWorkspace),ct.POINTER(CRoute)]
    lib.jp_plan.restype=ct.c_int
    from junctionpilot.graph import Graph,Node,Edge
    fixtures=json.loads(gzip.decompress(Path("reports/cases.json.gz").read_bytes()))
    queries=0
    for f in fixtures:
        raw=f["graph"];g=Graph([Node(**n) for n in raw["nodes"]],[Edge(**e) for e in raw["edges"]])
        cg=CGraph();cg.node_count=len(g.nodes);cg.edge_count=len(g.edges);cg.revision=g.revision
        for i,e in enumerate(g.edges):cg.edges[i]=CEdge(g.index[e.src],g.index[e.dst],g.heading(e),e.blocked,e.length_mm,e.width_mm,e.risk_permille)
        c=CCalibration(**f["calibration"])
        for policy_id,policy in enumerate(["distance","fastest","robust"]):
            workspace,output=CWorkspace(),CRoute()
            status=lib.jp_plan(ct.byref(cg),g.index[f["start"]],g.index[f["goal"]],f["heading"],policy_id,ct.byref(c),ct.byref(workspace),ct.byref(output))
            expected=f["routes"][policy]
            assert status==(0 if expected else 1),(f["seed"],policy,status)
            if expected:
                assert output.cost==expected["cost"]
                assert output.nominal_time_ms==expected["nominal_time_ms"]
                assert output.distance_mm==expected["distance_mm"]
                assert output.turn_count==expected["turn_count"]
                assert output.step_count==len(expected["steps"])
                for i,step in enumerate(expected["steps"]):
                    actual=output.steps[i]
                    assert g.nodes[actual.src].id==step["src"] and g.nodes[actual.dst].id==step["dst"]
                    assert actual.heading==step["heading"]
                    assert actual.travel_ms==step["travel_ms"] and actual.turn_ms==step["turn_ms"]
                    assert actual.risk_cost_ms==step["risk_cost_ms"]
            queries+=1
    lib.jp_motion_allowed.argtypes=[ct.POINTER(CGuard),ct.c_uint32,ct.c_uint8,ct.c_uint8,ct.c_uint8,ct.c_uint16]
    lib.jp_guard_arm.argtypes=[ct.POINTER(CGuard),ct.c_uint32,ct.c_uint8,ct.c_uint8,ct.c_uint8,ct.c_uint8]
    for revision,obstacle,recognized,pose in [(2,0,1,1),(1,1,1,1),(1,0,0,1),(1,0,1,0)]:
        guard=CGuard(1,0,pose,0)
        assert lib.jp_motion_allowed(ct.byref(guard),revision,1,obstacle,recognized,3)==0
        assert guard.latched_hold==1
        assert lib.jp_motion_allowed(ct.byref(guard),1,1,0,1,3)==0
    guard=CGuard(1,0,1,0)
    assert [lib.jp_motion_allowed(ct.byref(guard),1,0,0,1,3) for _ in range(3)]==[1,1,0]
    assert lib.jp_motion_allowed(ct.byref(guard),1,1,0,1,3)==0
    assert lib.jp_guard_arm(ct.byref(guard),1,1,1,1,1)==0
    assert lib.jp_guard_arm(ct.byref(guard),1,1,0,1,1)==1
    assert lib.jp_motion_allowed(ct.byref(guard),1,1,0,1,3)==1
    invalid=CGraph();invalid.node_count=65
    assert lib.jp_plan(ct.byref(invalid),0,1,1,1,ct.byref(c),ct.byref(CWorkspace()),ct.byref(CRoute()))==2
    assert lib.jp_plan(None,0,1,1,1,ct.byref(c),ct.byref(CWorkspace()),ct.byref(CRoute()))==2
    demo=build/("demo.exe" if os.name=="nt" else "demo")
    demo_command=[args.cc,"-std=c11","-Wall","-Wextra","-Werror","firmware/demo.c","firmware/jp_planner.c","firmware/jp_guard.c","-o",str(demo)]
    subprocess.run(demo_command,check=True)
    completed=subprocess.run([str(demo.resolve())],text=True,capture_output=True,check=True)
    log += ["CROSSCHECK: "+str(queries)+" / "+str(queries)+" exact route/cost/status matches",completed.stdout]
    receipt={"compiler":compiler_version,"platform":platform.platform(),"crosscheck_queries":queries,"all_matched":True,
             "workspace_bytes":ct.sizeof(CWorkspace),"graph_bytes":ct.sizeof(CGraph),"route_bytes":ct.sizeof(CRoute),
             "demo_exit_code":completed.returncode,"warning_policy":"-Wall -Wextra -Werror","hardware_run":False}
    Path("reports/c-verification.json").write_text(json.dumps(receipt,indent=2),encoding="utf-8")
    Path("reports/c-build.log").write_text("\n".join(log),encoding="utf-8")
    print(json.dumps(receipt,indent=2));print(completed.stdout)


if __name__=="__main__":main()
