// Model parameters and right-side spline basis are exported by MATLAB.
let model;
function setup(parameters) { model = parameters; }
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
function predict(x) {
  const gp = model.gp;
  let cd = gp.bias;
  for (let i = 0; i < gp.x.length; i++) {
    let squared = 0;
    for (let j = 0; j < 17; j++) squared += ((x[j] - gp.x[i][j]) / gp.kernel[j]) ** 2;
    const r = Math.sqrt(5 * squared);
    cd += gp.alpha[i] * gp.kernel[17] ** 2 * (1 + r + r*r/3) * Math.exp(-r);
  }
  return [cd, ...model.linear.map(s => s.bias + dot(x, s.weights))];
}

// Piecewise cubic Hermite interpolation, including the shape-preserving end slopes.
function leftRoof(x) {
  const nodes = [0, .349, .698, .940, 1];
  const y = [0, x[3], x[2], x[1], x[0]];
  const h = nodes.slice(1).map((v,i) => v - nodes[i]);
  const delta = h.map((v,i) => (y[i+1]-y[i])/v);
  function endpoint(h0,h1,d0,d1) {
    const m = ((2*h0+h1)*d0-h0*d1)/(h0+h1);
    if (Math.sign(m) !== Math.sign(d0)) return 0;
    if (Math.sign(d0) !== Math.sign(d1) && Math.abs(m) > 3*Math.abs(d0)) return 3*d0;
    return m;
  }
  const slope = [endpoint(h[0],h[1],delta[0],delta[1])];
  for (let i = 1; i < 4; i++) {
    const a = 2*h[i] + h[i-1], b = h[i] + 2*h[i-1];
    slope.push(delta[i-1]*delta[i] > 0 ? (a+b)/(a/delta[i-1]+b/delta[i]) : 0);
  }
  slope.push(endpoint(h[3],h[2],delta[3],delta[2]));
  return t => {
    const i = t <= nodes[1] ? 0 : t <= nodes[2] ? 1 : t <= nodes[3] ? 2 : 3;
    const u = (t-nodes[i])/h[i], v = 1-u;
    return (1+2*u)*v*v*y[i] + u*v*v*h[i]*slope[i]
      + (3-2*u)*u*u*y[i+1] - u*u*v*h[i]*slope[i+1];
  };
}
const lengthBase = 41.429;
function ends(x,left,right) {
  const offset = x[14]+x[15];
  const lx = x[13]-x[11], rx = x[13]+x[12];
  const ly = offset+left(1), ry = offset+x[10];
  const leftSlope = (left(198/199)-left(1))/Math.max(x[11]/199, 1e-6);
  const rightSlope = (x[10]-dot(model.rightDense[198],right))/Math.max(x[12]/199, 1e-6);
  return {lx,rx,ly,ry,lc:ly-lx*leftSlope,rc:ry+(lengthBase-rx)*rightSlope};
}
function contour(x) {
  const left = leftRoof(x), right = [0,...x.slice(4,11)];
  const {lx,rx,ly,ry,lc,rc} = ends(x,left,right);
  const scale = (x[16]>1000 ? x[16]/lengthBase : x[16])/1000;
  const xs = [], ys = [];
  function point(a,b) { xs.push(a*scale); ys.push(b*scale); }
  function bezier(t,topX,topY,baseX,controlY) {
    const v=1-t;
    point(v*v*topX+(2*v*t+t*t)*baseX, v*v*topY+2*v*t*controlY+t*t*x[15]);
  }
  for (let i=49;i>=0;i--) bezier(i/49,lx,ly,0,lc);
  for (let i=199;i>=0;i--) point(x[13]-i/199*x[11],left(i/199)+x[14]+x[15]);
  for (let i=0;i<200;i++) point(x[13]+i/199*x[12],dot(model.rightDense[i],right)+x[14]+x[15]);
  const height = Math.max(...ys.slice(50));
  for (let i=0;i<50;i++) bezier(i/49,rx,ry,lengthBase,rc);
  point(0,x[15]);
  return {success:true,x:xs,y:ys,roof_h:height,wheelbase:lengthBase*scale};
}
function convexityViolation(values) {
  let worst = 0;
  for (let i=2;i<values.length;i++)
    worst = Math.max(worst,values[i]-2*values[i-1]+values[i-2]-1e-5);
  return worst;
}
function evaluate(x, scores) {
  const predictions = predict(x);
  let violation = 0, error = 0;
  predictions.forEach((v,i) => {
    const fraction = (v-model.mins[i])/(model.maxs[i]-model.mins[i]);
    violation += Math.max(0,-.05-fraction,fraction-1.05);
    if (i) error += (fraction-scores[i-1]/10)**2;
  });
  const left = leftRoof(x), right = [0,...x.slice(4,11)];
  const e = ends(x,left,right);
  violation += Math.max(0,x[15]-e.lc) + Math.max(0,x[15]-e.rc)
    + Math.max(0,-e.lx) + Math.max(0,e.rx-lengthBase)
    + convexityViolation(Array.from({length:40},(_,i) => left(i/39)))
    + convexityViolation(model.rightCheck.map(row => dot(row,right)));
  return {x, cd:predictions[0], error, violation};
}

// Elitist non-dominated sorting with crowding-distance selection.
function select(candidates, count) {
  const feasible = candidates.filter(p=>p.violation===0).sort((a,b)=>a.cd-b.cd || a.error-b.error);
  const layers = [];
  for (const p of feasible) {
    let rank = 0;
    while (rank < layers.length && layers[rank].at(-1).error <= p.error) rank++;
    (layers[rank] ??= []).push(p);
  }
  const next = [];
  for (const layer of layers) {
    if (next.length + layer.length <= count) next.push(...layer);
    else {
      const cdSpan = layer.at(-1).cd-layer[0].cd || 1;
      const errSpan = layer[0].error-layer.at(-1).error || 1;
      layer.forEach((p,i) => {
        p.distance = i===0 || i===layer.length-1 ? Infinity :
          (layer[i+1].cd-layer[i-1].cd)/cdSpan+(layer[i-1].error-layer[i+1].error)/errSpan;
      });
      next.push(...[...layer].sort((a,b)=>b.distance-a.distance).slice(0,count-next.length));
      break;
    }
  }
  if (next.length < count) next.push(...candidates.filter(p=>p.violation>0)
    .sort((a,b)=>a.violation-b.violation).slice(0,count-next.length));
  return {population:next,front:layers[0] || []};
}
function optimize(scores, report=()=>{}) {
  const size=200, generations=50;
  const randomIndex = n=>Math.floor(Math.random()*n);
  const span = model.lb.map((v,i)=>model.ub[i]-v);
  let initial = model.gp.x.map(x=>evaluate(x,scores));
  while (initial.length<size) {
    const a=model.gp.x[randomIndex(model.gp.x.length)], b=model.gp.x[randomIndex(model.gp.x.length)];
    const t=Math.random();
    initial.push(evaluate(a.map((v,i)=>v+t*(b[i]-v)),scores));
  }
  let state = select(initial,size);
  for (let gen=0;gen<generations;gen++) {
    const pop=state.population;
    const parent=()=>pop[Math.min(randomIndex(pop.length),randomIndex(pop.length))].x;
    const children=[];
    for (let k=0;k<size;k++) {
      const a=parent(), b=parent();
      const t=Math.random()*1.4-.2;
      const x=a.map((v,i)=>{
        let value=v+t*(b[i]-v);
        if (Math.random()<.2) value+=(Math.random()-.5)*(.18-.14*gen/generations)*span[i];
        return Math.max(model.lb[i],Math.min(model.ub[i],value));
      });
      children.push(evaluate(x,scores));
    }
    state=select([...pop,...children],size);
    if (gen%5===0) report(gen+1);
  }
  if (!state.front.length) throw Error('未找到满足约束的方案，请再试一次。');
  const front=state.front, best=front[0];
  return {...contour(best.x),cd:best.cd,emo_score:best.error,
    pareto_designs:front.map(p=>p.x),pareto_cd:front.map(p=>p.cd),pareto_emo:front.map(p=>p.error)};
}
