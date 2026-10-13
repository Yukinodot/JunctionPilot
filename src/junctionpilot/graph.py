from dataclasses import dataclass, replace
import json
from pathlib import Path

HEADINGS = ("N", "E", "S", "W")


@dataclass(frozen=True)
class Node:
    id: str
    x: int
    y: int


@dataclass(frozen=True)
class Edge:
    src: str
    dst: str
    length_mm: int
    width_mm: int = 300
    risk_permille: int = 0
    blocked: bool = False


class Graph:
    def __init__(self, nodes, edges, revision=0, title="Practice map"):
        self.nodes = list(nodes)
        self.edges = list(edges)
        self.revision, self.title = revision, title
        self.index = {n.id:i for i,n in enumerate(self.nodes)}
        if not 1 <= len(self.nodes) <= 64 or len(self.index) != len(self.nodes):
            raise ValueError("Require 1..64 uniquely named nodes")
        if len(self.edges) > 256:
            raise ValueError("Maximum 256 directed edges")
        if not isinstance(revision,int) or not 0<=revision<=0xffffffff:
            raise ValueError("Revision must fit uint32_t")
        if any(not isinstance(n.id,str) or not n.id or not all(isinstance(v,int) and not isinstance(v,bool) for v in (n.x,n.y)) for n in self.nodes):
            raise ValueError("Node IDs must be nonempty strings; coordinates must be integers")
        pairs=set()
        for e in self.edges:
            if e.src not in self.index or e.dst not in self.index or (e.src,e.dst) in pairs:
                raise ValueError("Invalid/duplicate edge endpoints")
            pairs.add((e.src,e.dst))
            if not all(isinstance(v,int) and not isinstance(v,bool) for v in (e.length_mm,e.width_mm,e.risk_permille)):
                raise ValueError("Edge parameters must be integers")
            if not 1 <= e.length_mm <= 10000 or not 1 <= e.width_mm <= 10000 or not 0 <= e.risk_permille <= 1000:
                raise ValueError("Invalid edge length, width, or risk")
            if not isinstance(e.blocked,bool):raise ValueError("blocked must be boolean")
            self.heading(e)
            a,b=self.nodes[self.index[e.src]],self.nodes[self.index[e.dst]]
            if any(n.id not in (e.src,e.dst) and ((a.x==b.x==n.x and min(a.y,b.y)<n.y<max(a.y,b.y)) or
                   (a.y==b.y==n.y and min(a.x,b.x)<n.x<max(a.x,b.x))) for n in self.nodes):
                raise ValueError("Split an edge at every intermediate junction")
        self.adj = {n.id:[] for n in self.nodes}
        for i,e in enumerate(self.edges):
            self.adj[e.src].append((i,e))

    def heading(self, edge):
        a,b = self.nodes[self.index[edge.src]],self.nodes[self.index[edge.dst]]
        dx,dy=b.x-a.x,b.y-a.y
        if dx == 0 and dy > 0:return 0
        if dy == 0 and dx > 0:return 1
        if dx == 0 and dy < 0:return 2
        if dy == 0 and dx < 0:return 3
        raise ValueError("Only nonzero axis-aligned segments are supported")

    def with_block(self, src, dst, blocked=True, both=True):
        if (src,dst) not in {(e.src,e.dst) for e in self.edges}:
            raise ValueError("Cannot update an unknown edge")
        edges=[replace(e,blocked=blocked) if (e.src,e.dst)==(src,dst) or
               (both and (e.src,e.dst)==(dst,src)) else e for e in self.edges]
        return Graph(self.nodes,edges,self.revision+1,self.title)

    def to_dict(self):
        from dataclasses import asdict
        return {"title":self.title,"revision":self.revision,"nodes":[asdict(n) for n in self.nodes],
                "edges":[asdict(e) for e in self.edges]}


def load_graph(path):
    raw=json.loads(Path(path).read_text(encoding="utf-8"))
    edges=[]
    for row in raw["edges"]:
        row=dict(row)
        bidirectional=row.pop("bidirectional",False)
        edge=Edge(**row)
        edges.append(edge)
        if bidirectional:edges.append(replace(edge,src=edge.dst,dst=edge.src))
    return Graph([Node(**n) for n in raw["nodes"]],edges,raw.get("revision",0),raw.get("title","Practice map"))
