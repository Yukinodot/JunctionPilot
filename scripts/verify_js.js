"use strict";
const fs=require('node:fs'),assert=require('node:assert/strict');
const {plan}=require('../demo/planner.js');
const cases=JSON.parse(require('node:zlib').gunzipSync(fs.readFileSync('reports/cases.json.gz')).toString('utf8'));
let queries=0;
for(const f of cases)for(const policy of ['distance','fastest','robust']){
  const actual=plan(f.graph,f.start,f.goal,f.heading,policy,f.calibration),expected=f.routes[policy];
  assert.equal(actual===null,expected===null,`seed=${f.seed} policy=${policy}`);
  if(expected){
    for(const key of ['cost','nominal_time_ms','distance_mm','turn_count','risk_proxy'])assert.equal(actual[key],expected[key]);
    assert.deepEqual(actual.steps,expected.steps);
  }
  queries++;
}
fs.writeFileSync('reports/js-verification.json',JSON.stringify({queries,all_matched:true,node:process.version,scope:'Exact routes and integer costs against saved Python runs'},null,2));
console.log(`${queries}/${queries} JavaScript/Python exact route/cost/status checks passed`);
