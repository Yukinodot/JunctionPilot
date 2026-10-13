/* Same integer model as the Python/C planners, executed locally in the viewer. */
(function(root){
"use strict";
const defaults={speed_mm_s:300,left_ms:850,right_ms:650,uturn_ms:1600,settle_ms:100,vehicle_width_mm:150,clearance_mm:20,risk_penalty_ms:8000};
function normalize(raw){
  const nodes=raw.nodes.map(n=>({...n})), index=new Map(nodes.map((n,i)=>[n.id,i]));
  if(nodes.length<1||nodes.length>64||index.size!==nodes.length||nodes.some(n=>!n.id||!Number.isInteger(n.x)||!Number.isInteger(n.y)))throw Error("无效节点：需要唯一编号和整数坐标");
  const edges=[];for(const input of raw.edges){
    const edge={width_mm:300,risk_permille:0,blocked:false,...input};delete edge.bidirectional;edges.push(edge);
    if(input.bidirectional)edges.push({...edge,src:edge.dst,dst:edge.src});
  }
  if(edges.length>256)throw Error("最多 256 条有向道路");
  const pairs=new Set();
  for(const e of edges){
    if(!index.has(e.src)||!index.has(e.dst)||pairs.has(e.src+"\0"+e.dst))throw Error("无效或重复道路");pairs.add(e.src+"\0"+e.dst);
    if(!Number.isInteger(e.length_mm)||e.length_mm<1||e.length_mm>10000||!Number.isInteger(e.width_mm)||e.width_mm<1||e.width_mm>10000||!Number.isInteger(e.risk_permille)||e.risk_permille<0||e.risk_permille>1000||typeof e.blocked!=="boolean")throw Error("道路属性超出范围");
    const a=nodes[index.get(e.src)],b=nodes[index.get(e.dst)],dx=b.x-a.x,dy=b.y-a.y;
    if(dx===0&&dy>0)e.heading=0;else if(dy===0&&dx>0)e.heading=1;else if(dx===0&&dy<0)e.heading=2;else if(dy===0&&dx<0)e.heading=3;else throw Error("仅支持横向或纵向道路");
    if(nodes.some(n=>n.id!==e.src&&n.id!==e.dst&&((a.x===b.x&&n.x===a.x&&n.y>Math.min(a.y,b.y)&&n.y<Math.max(a.y,b.y))||(a.y===b.y&&n.y===a.y&&n.x>Math.min(a.x,b.x)&&n.x<Math.max(a.x,b.x)))))throw Error("道路须在中间路口处分段");
  }
  return {nodes,edges,index,revision:raw.revision||0,title:raw.title||"练习地图"};
}
function plan(raw,start,goal,heading=1,policy="fastest",input={}){
  const g=raw.index?raw:normalize(raw),c={...defaults,...input};
  if(!g.index.has(start)||!g.index.has(goal)||!Number.isInteger(heading)||heading<0||heading>3||!["distance","fastest","robust"].includes(policy))throw Error("无效起点、终点、车头方向或策略");
  if(Object.values(c).some(v=>!Number.isInteger(v)||v<0)||c.speed_mm_s<50||c.speed_mm_s>5000||c.vehicle_width_mm<1||c.vehicle_width_mm>10000||c.clearance_mm>5000||["left_ms","right_ms","uturn_ms","settle_ms","risk_penalty_ms"].some(k=>c[k]>60000))throw Error("标定值超出范围");
  const count=4*g.nodes.length,initial=4*g.index.get(start)+heading;
  const dist=Array(count).fill(Infinity),parent=Array(count).fill(null),settled=Array(count).fill(false);dist[initial]=0;
  for(let iteration=0;iteration<count;iteration++){
    let state=-1,best=Infinity;for(let i=0;i<count;i++)if(!settled[i]&&dist[i]<best){state=i;best=dist[i];}
    if(state<0)return null;
    const node=g.nodes[Math.floor(state/4)].id;
    if(node===goal){
      const steps=[];for(let end=state;end!==initial;end=parent[end].state)steps.push(parent[end].step);steps.reverse();
      return {start,goal,start_heading:heading,policy,revision:g.revision,cost:best,steps,
        nominal_time_ms:steps.reduce((a,s)=>a+s.travel_ms+s.turn_ms,0),distance_mm:steps.reduce((a,s)=>a+s.length_mm,0),
        turn_count:steps.filter(s=>s.action!=="STRAIGHT").length,risk_proxy:steps.reduce((a,s)=>a+s.risk_permille,0),
        assumed_failure_probability:1-steps.reduce((a,s)=>a*(1-s.risk_permille/1000),1),cost_unit:policy==="distance"?"mm":"ms"};
    }
    settled[state]=true;
    for(const e of g.edges){
      if(e.src!==node||e.blocked||e.width_mm<c.vehicle_width_mm+2*c.clearance_mm)continue;
      const delta=(e.heading-state%4+4)%4;
      const turn=[0,c.right_ms,c.uturn_ms,c.left_ms][delta]+(delta?c.settle_ms:0);
      const travel=Math.floor((e.length_mm*1000+c.speed_mm_s-1)/c.speed_mm_s),risk=Math.floor((e.risk_permille*c.risk_penalty_ms+999)/1000);
      const step={src:e.src,dst:e.dst,heading:e.heading,action:["STRAIGHT","RIGHT","UTURN","LEFT"][delta],travel_ms:travel,turn_ms:turn,risk_cost_ms:risk,length_mm:e.length_mm,risk_permille:e.risk_permille};
      const cost=policy==="distance"?e.length_mm:travel+turn+(policy==="robust"?risk:0),next=4*g.index.get(e.dst)+e.heading;
      if(best+cost<dist[next]){dist[next]=best+cost;parent[next]={state,step};}
    }
  }
  return null;
}
const api={defaults,normalize,plan};
if(typeof module!=="undefined"&&module.exports)module.exports=api;else root.JunctionPilot=api;
})(typeof globalThis!=="undefined"?globalThis:this);
