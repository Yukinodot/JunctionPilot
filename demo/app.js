"use strict";
const $=id=>document.getElementById(id),SVG="http://www.w3.org/2000/svg";
const labels={distance:"最短距离",fastest:"预计最快",robust:"风险加权"},headings=["北 N","东 E","南 S","西 W"];
let graph=null,currentNode="S",currentHeading=1,goal="G",route=null,routes={},calibration={...JP_DATA.calibration};
let moving=false,poseConfirmed=true,token=0,carPosition=null,events=[];
function option(parent,value,text){const el=document.createElement("option");el.value=value;el.textContent=text;parent.append(el);}
function svgEl(tag,attrs={}){const el=document.createElementNS(SVG,tag);for(const [key,value] of Object.entries(attrs))el.setAttribute(key,String(value));return el;}
function log(message){events.unshift(`r${graph?.revision||0} · ${message}`);events=events.slice(0,12);$("event-log").replaceChildren();for(const message of events){const el=document.createElement("div");el.className="event";el.textContent=message;$("event-log").append(el);}}
function point(id){const n=graph.nodes[graph.index.get(id)],xs=graph.nodes.map(n=>n.x),ys=graph.nodes.map(n=>n.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);return {x:85+(n.x-minX)*730/Math.max(1,maxX-minX),y:350-(n.y-minY)*270/Math.max(1,maxY-minY)};}
function draw(){
  const svg=$("map-svg");svg.setAttribute("viewBox","0 0 900 420");svg.replaceChildren();
  for(let x=35;x<900;x+=45)svg.append(svgEl("line",{x1:x,y1:30,x2:x,y2:390,stroke:"#1c2d3b","stroke-width":1}));
  for(let y=30;y<420;y+=45)svg.append(svgEl("line",{x1:35,y1:y,x2:865,y2:y,stroke:"#1c2d3b","stroke-width":1}));
  const used=new Set(route?.steps.map(s=>[s.src,s.dst].sort().join("|"))||[]),seen=new Set();
  for(const e of graph.edges){
    const key=[e.src,e.dst].sort().join("|");if(seen.has(key))continue;seen.add(key);
    const a=point(e.src),b=point(e.dst),selected=used.has(key),group=svgEl("g",{class:"edge",role:"button",tabindex:0,"aria-label":`道路 ${e.src} 与 ${e.dst}，${e.blocked?"已阻塞":"可通行"}`,"aria-pressed":e.blocked});
    const color=e.blocked?"#f77d86":selected?"#78baff":e.risk_permille>=100?"#efc879":"#4c6273";
    group.append(svgEl("rect",a.x===b.x?{x:a.x-22,y:Math.min(a.y,b.y)-20,width:135,height:Math.abs(a.y-b.y)+40,fill:"transparent"}:{x:Math.min(a.x,b.x)-20,y:a.y-30,width:Math.abs(a.x-b.x)+40,height:50,fill:"transparent"}));
    group.append(svgEl("line",{x1:a.x,y1:a.y,x2:b.x,y2:b.y,stroke:"transparent","stroke-width":28}));
    group.append(svgEl("line",{x1:a.x,y1:a.y,x2:b.x,y2:b.y,stroke:color,"stroke-width":selected?6:3,"stroke-dasharray":e.blocked?"9 6":"none"}));
    const text=svgEl("text",{x:(a.x+b.x)/2+(a.x===b.x?10:0),y:(a.y+b.y)/2+(a.x===b.x?0:-12),fill:color,"font-size":13,"text-anchor":a.x===b.x?"start":"middle"});text.textContent=`${e.length_mm} / ${e.risk_permille}`;group.append(text);
    group.addEventListener("click",()=>toggleEdge(e.src,e.dst));group.addEventListener("keydown",event=>{if(event.key==="Enter"||event.key===" "){event.preventDefault();toggleEdge(e.src,e.dst);}});svg.append(group);
  }
  for(const n of graph.nodes){const p=point(n.id);svg.append(svgEl("circle",{cx:p.x,cy:p.y,r:11,fill:"#142632",stroke:n.id===goal?"#efc879":"#a2b5c4","stroke-width":2}));const t=svgEl("text",{x:p.x,y:p.y+30,fill:"#e7f0f6","font-size":16,"text-anchor":"middle"});t.textContent=n.id===goal?`${n.id} · 目的地`:n.id;svg.append(t);}
  const p=carPosition||point(currentNode),car=svgEl("g",{transform:`translate(${p.x},${p.y}) rotate(${[-90,0,90,180][currentHeading]})`});car.append(svgEl("polygon",{points:"15,0 -10,-10 -6,0 -10,10",fill:"#73ddc4",stroke:"#0d1720","stroke-width":2}));svg.append(car);
}
function showComparison(){
  $("comparison").replaceChildren();for(const policy of ["distance","fastest","robust"]){
    const r=routes[policy],row=document.createElement("tr");if(policy===$("policy").value)row.className="active";
    const values=r?[labels[policy],[r.start,...r.steps.map(s=>s.dst)].join(" → "),`${r.distance_mm} mm`,r.turn_count,`${(r.nominal_time_ms/1000).toFixed(3)} s`,r.risk_proxy]:[labels[policy],poseConfirmed?"无可通行路线":"位置未确认","—","—","—","—"];
    for(const v of values){const el=document.createElement("td");el.textContent=v;row.append(el);}$("comparison").append(row);
  }
}
function replan(){
  $("localization").hidden=poseConfirmed;$("title").textContent=graph.title;$("pose").textContent=poseConfirmed?`${currentNode} · ${headings[currentHeading]} · r${graph.revision}`:"位置未确认 · 保持停车";
  if(!poseConfirmed){route=null;routes={};$("status").textContent="停在路段中间。请确认所在路口与车头后再规划。";}
  else{
    routes={};for(const policy of ["distance","fastest","robust"])routes[policy]=JunctionPilot.plan(graph,currentNode,goal,currentHeading,policy,calibration);
    route=routes[$("policy").value];
    $("status").textContent=!route?"无可通行路线：保持停车。":route.steps.length===0?"已到达目的节点，可选择返航任务。":`${labels[route.policy]}：${[route.start,...route.steps.map(s=>s.dst)].join(" → ")}`;
  }
  $("distance").textContent=route?`${route.distance_mm} mm`:"—";$("time").textContent=route?`${(route.nominal_time_ms/1000).toFixed(3)} s`:"—";$("turns").textContent=route?route.turn_count:"—";
  $("step").disabled=moving||!poseConfirmed||!route||route.steps.length===0;$("return").disabled=moving||!poseConfirmed;
  showComparison();draw();
}
function stopIfMoving(){
  if(moving){token++;moving=false;poseConfirmed=false;log("HOLD：路段中停止，等待位置确认");}
}
function toggleEdge(src,dst){
  stopIfMoving();const edge=graph.edges.find(e=>e.src===src&&e.dst===dst),blocked=!edge.blocked;
  for(const e of graph.edges)if((e.src===src&&e.dst===dst)||(e.src===dst&&e.dst===src))e.blocked=blocked;
  graph.revision++;log(`${blocked?"阻塞":"恢复"} ${src} ↔ ${dst}，旧路线作废`);replan();
}
function reset(){
  token++;moving=false;poseConfirmed=true;currentNode="S";currentHeading=Number($("heading").value);carPosition=null;goal=$("goal").value;log("手动重置：从起点开始新实验");replan();
}
function changeMap(){
  token++;moving=false;poseConfirmed=true;events=[];graph=JunctionPilot.normalize(JSON.parse(JSON.stringify(JP_DATA.maps[Number($("map").value)])));
  $("goal").replaceChildren();$("pose-node").replaceChildren();for(const n of graph.nodes){option($("goal"),n.id,n.id);option($("pose-node"),n.id,n.id);}$("goal").value="G";reset();
}
function advance(){
  if(moving||!poseConfirmed||!route||!route.steps.length)return;
  if(route.revision!==graph.revision){log("HOLD：拒绝地图版本不一致的路线");replan();return;}
  const step=route.steps[0],a=point(step.src),b=point(step.dst),mine=++token,begin=performance.now();moving=true;poseConfirmed=false;
  $("step").disabled=true;$("return").disabled=true;log(`执行 ${step.action}，${step.src} → ${step.dst}，名义 ${step.turn_ms+step.travel_ms} ms`);
  $("status").textContent="路段执行动画（示意播放），到达后确认路口。";
  function frame(now){
    if(mine!==token)return;
    const f=Math.min(1,(now-begin)/1800);carPosition={x:a.x+(b.x-a.x)*f,y:a.y+(b.y-a.y)*f};draw();
    if(f<1)requestAnimationFrame(frame);else{moving=false;poseConfirmed=true;currentNode=step.dst;currentHeading=step.heading;carPosition=null;log(`确认到达 ${currentNode}，车头 ${headings[currentHeading]}`);replan();}
  }
  requestAnimationFrame(frame);
}
JP_DATA.maps.forEach((map,i)=>option($("map"),String(i),map.title));$("map").value=String(JP_DATA.maps.findIndex(m=>m.nodes.some(n=>n.id==="U")));
$("map").addEventListener("change",changeMap);
$("policy").addEventListener("change",()=>{stopIfMoving();log(`切换策略 ${labels[$("policy").value]}`);replan();});
$("goal").addEventListener("change",()=>{stopIfMoving();goal=$("goal").value;log(`修改目的节点 ${goal}`);replan();});
$("heading").addEventListener("change",()=>reset());
for(const id of ["left","right","risk"])$(id).addEventListener("input",()=>{
  stopIfMoving();$(id+"-value").textContent=$(id).value;calibration[{left:"left_ms",right:"right_ms",risk:"risk_penalty_ms"}[id]]=Number($(id).value);replan();
});
$("step").addEventListener("click",advance);$("reset").addEventListener("click",reset);
$("return").addEventListener("click",()=>{if(!poseConfirmed)return;goal="S";$("goal").value="S";log("返航任务：保留当前车头方向");replan();});
$("confirm-pose").addEventListener("click",()=>{token++;moving=false;currentNode=$("pose-node").value;currentHeading=Number($("pose-heading").value);poseConfirmed=true;carPosition=null;log(`人工确认路口 ${currentNode} 与车头，允许重新规划`);replan();});
changeMap();
