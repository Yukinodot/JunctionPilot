"""Generate a C topology header from a validated practice map."""
import argparse
from pathlib import Path
from junctionpilot.graph import load_graph

p=argparse.ArgumentParser()
p.add_argument("map",type=Path)
p.add_argument("--output",type=Path,default=Path("outputs/map_generated.h"))
args=p.parse_args()
g=load_graph(args.map)
lines=["/* Generated topology; coordinates use N=+y, E=+x. */","#ifndef JP_MAP_GENERATED_H","#define JP_MAP_GENERATED_H",'#include "jp_planner.h"',
       "static const jp_graph jp_generated_map = {",f"    .node_count = {len(g.nodes)}, .edge_count = {len(g.edges)}, .revision = {g.revision},","    .edges = {"]
for edge in g.edges:
    lines.append("        {%d,%d,%d,%d,%d,%d,%d},"%(g.index[edge.src],g.index[edge.dst],g.heading(edge),edge.blocked,edge.length_mm,edge.width_mm,edge.risk_permille))
lines.extend(["    }","};","#endif",""])
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text("\n".join(lines),encoding="utf-8")
print("Node indices:",g.index)
