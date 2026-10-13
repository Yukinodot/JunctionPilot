import json
from pathlib import Path
from dataclasses import asdict
from junctionpilot.graph import load_graph
from junctionpilot.planner import Calibration,plan
from junctionpilot.mission import Mission

maps=[];examples=[]
for path in sorted(Path("maps").glob("*.json")):
    g=load_graph(path);maps.append(g.to_dict())
    examples.append({"map":path.name,"routes":{p:plan(g,"S","G",1,p).to_dict() for p in ["distance","fastest","robust"]}})
g=load_graph("maps/turn_tradeoff.json")
m=Mission(g,"S",1,"G",Calibration());m.prepare();step=m.next_step();m.obstacle(step.src,step.dst);m.confirm_pose("S",1)
Path("reports/mission-events.json").write_text(json.dumps(m.log,indent=2),encoding="utf-8")
Path("reports/examples.json").write_text(json.dumps(examples,indent=2),encoding="utf-8")
payload={"maps":maps,"calibration":asdict(Calibration()),"summary":json.loads(Path("reports/summary.json").read_text())}
Path("demo/data.js").write_text("window.JP_DATA = "+json.dumps(payload,ensure_ascii=False,separators=(",",":"))+";\n",encoding="utf-8")
print("Demo maps and modeled mission events generated")
