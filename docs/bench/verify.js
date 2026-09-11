const M=require('./planner.js'),fs=require('fs');
const scenes=[
 [{x:5,y:7,r:1.1},{x:9,y:4.2,r:1.3},{x:9,y:10.0,r:1.3},{x:13,y:7,r:1.5},{x:17,y:4.5,r:1.2},{x:17,y:9.5,r:1.2}],
 [{x:7,y:3.0,r:2.6},{x:7,y:11.0,r:2.6},{x:13,y:3.2,r:2.6},{x:13,y:10.8,r:2.6},{x:18,y:7,r:1.0}],
 [{x:4.5,y:5.0,r:.95},{x:4.5,y:9.0,r:.95},{x:7.5,y:7.0,r:.95},{x:7.5,y:3.0,r:.95},{x:7.5,y:11.0,r:.95},
  {x:10.5,y:5.0,r:.95},{x:10.5,y:9.0,r:.95},{x:13.5,y:7.0,r:.95},{x:13.5,y:3.2,r:.95},{x:13.5,y:10.8,r:.95},
  {x:16.5,y:5.0,r:.95},{x:16.5,y:9.0,r:.95}]];
// C++ 결과
const ref={};
for(const l of fs.readFileSync(__dirname+'/results.csv','utf8').trim().split('\n').slice(1)){
  const f=l.split(','); ref[f[0]+f[1][0]]={len:+f[2],jerk2:+f[3],snap2:+f[4],minClr:+f[5],corrViol:+f[6],penViol:+f[7],vmax:+f[9],amax:+f[10],T:+f[11]};
}
const F=(a,b,d)=>`${a.toFixed(d)}/${b.toFixed(d)}`;
for(let s=0;s<3;s++){
  const S={obs:scenes[s],start:[1.5,7.0],goal:[20.5,7.0],xmin:0,xmax:22,ymin:0,ymax:14};
  const t0=Date.now(); const r=M.planAll(S,{}); const ms=Date.now()-t0;
  if(!r.ok){console.log(s,r.err);continue;}
  console.log(`\n=== 장면 ${s}  (총 ${ms}ms, A ${r.A.ms.toFixed(1)}ms iter${r.A.iter}, D ${r.D?r.D.ms.toFixed(1):'-'}ms) ===`);
  console.log('        len(JS/C++)   jerk2         snap2        minClr      penViol      vmax        amax         T');
  for(const k of ['A','B','C','D']){
    if(!r[k]){console.log(k,'없음');continue;}
    const m=r[k].m, R=ref[s+k];
    console.log(`  ${k}  ${F(m.len,R.len,2)}  ${F(m.jerk2,R.jerk2,1)}  ${F(m.snap2,R.snap2,0)}  ${F(m.minClr,R.minClr,3)}  ${F(m.penViol,R.penViol,4)}  ${F(m.vmax,R.vmax,2)}  ${F(m.amax,R.amax,2)}  ${F(m.T,R.T,2)}`);
  }
}
