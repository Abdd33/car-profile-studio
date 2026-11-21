async function generateDesigns(scores, progress) {
  const worker = new Worker('./optimizer-worker.js');
  try {
    return await new Promise((resolve,reject) => {
      worker.onmessage = ({data}) => {
        if (data.error) reject(new Error(data.error));
        else if (data.result) resolve(data.result);
        else progress(data.generation);
      };
      worker.onerror = () => reject(new Error('优化模块加载失败，请刷新后重试。'));
      worker.postMessage(scores);
    });
  } finally {
    worker.terminate();
  }
}
let profileModelReady;
function loadProfileModel() {
  profileModelReady ??= fetch('./assets/model.json').then(async response => {
    if (!response.ok) throw Error('轮廓模型加载失败');
    setup(await response.json());
  }).catch(error => { profileModelReady = undefined; throw error; });
  return profileModelReady;
}
async function cadBlob(format, xs, ys) {
  const pts = xs.map((x,i)=>[x,ys[i],0]).filter((p,i,all) =>
    !i || Math.hypot(p[0]-all[i-1][0],p[1]-all[i-1][1])>1e-12);
  if (format === 'dxf') {
    const {DxfWriter,Units,LWPolylineFlags} = await import('./assets/dxf.mjs');
    const doc = new DxfWriter();
    doc.setUnits(Units.Meters);
    doc.addLayer('CarProfile_Polyline',1,'Continuous');
    doc.addLayer('CarProfile_Spline',3,'Continuous');
    doc.addLWPolyline(pts.slice(0,-1).map(([x,y])=>({point:{x,y}})),
      {flags:LWPolylineFlags.Closed,layerName:'CarProfile_Polyline'});
    // A piecewise cubic interpolant with a knot at each sampled profile point.
    const ctrl = [pts[0]], knots = [0,0,0,0];
    const distances = pts.slice(1).map((p,i)=>Math.hypot(p[0]-pts[i][0],p[1]-pts[i][1]));
    function tangent(i,axis) {
      const prev = Math.max(0,i-1), next = Math.min(pts.length-1,i+1);
      return (pts[next][axis]-pts[prev][axis])/((distances[i-1]||0)+(distances[i]||0));
    }
    let t=0;
    for (let i=0;i<distances.length;i++) {
      const h=distances[i];
      ctrl.push(pts[i].map((v,j)=>v+h*tangent(i,j)/3),
        pts[i+1].map((v,j)=>v-h*tangent(i+1,j)/3),pts[i+1]);
      t+=h;
      knots.push(t,t,t);
    }
    knots.push(t);
    doc.addSpline({controlPoints:ctrl.map(([x,y,z])=>({x,y,z})),knots,degreeCurve:3},
      {layerName:'CarProfile_Spline'});
    return new Blob([doc.stringify()],{type:'application/dxf'});
  }
  const rhino = await getRhino();
  const doc = new rhino.File3dm();
  try {
    doc.settings().modelUnitSystem = rhino.UnitSystem.Meters;
    function addCurve(name,color,curve) {
      const attr = new rhino.ObjectAttributes();
      try {
        attr.layerIndex = doc.layers().addLayer(name,color);
        doc.objects().add(curve,attr);
      } finally {
        curve.delete(); attr.delete();
      }
    }
    addCurve('CarProfile_Polyline',{r:255,g:0,b:0,a:255},new rhino.PolylineCurve(pts));
    addCurve('CarProfile_NURBS',{r:0,g:128,b:0,a:255},rhino.NurbsCurve.create(false,3,pts));
    return new Blob([doc.toByteArray()],{type:'application/octet-stream'});
  } finally { doc.delete(); }
}
let rhinoReady;
function getRhino() {
  rhinoReady ??= rhino3dm({locateFile:name=>new URL('./assets/'+name,document.baseURI).href})
    .catch(error=>{rhinoReady=undefined; throw error;});
  return rhinoReady;
}
