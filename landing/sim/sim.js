'use strict';
const $=id=>document.getElementById(id), world=$('world'), ctx=world.getContext('2d'), camera=$('camera'), cam=camera.getContext('2d');
const HOME=[0,0,0];
let state={p:[0,0,.5],v:[0,0,0],phase:'READY',time:0,lock:false,error:null}, paused=false, orbit=.8, elevation=.64, zoom=1, trail=[];
let observation=null, frameImage=null, busy=false, pendingAction='reset', networkError='', running=false;
const session=crypto.randomUUID(), clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
let autoDemo=false;
const heldPointers=new Map(), heldKeys=new Set();
function clearControls(){heldPointers.clear();heldKeys.clear();}
function requestAction(action){clearControls();pendingAction=action;paused=false;if(action==='start')running=true;else if(action==='reset')running=false;}
function manualInput(){
 const input=[0,0,0];for(const vector of [...heldPointers.values(),...Array.from(heldKeys,key=>keyControls[key])])for(let i=0;i<3;i++)input[i]+=vector[i];return input.map(x=>clamp(x,-1,1));
}
const keyControls={ArrowUp:[0,-1,0],ArrowDown:[0,1,0],ArrowLeft:[-1,0,0],ArrowRight:[1,0,0],KeyW:[0,0,1],KeyS:[0,0,-1]};
for(const [id,vector] of [['cmdUp',[0,-1,0]],['cmdDown',[0,1,0]],['cmdLeft',[-1,0,0]],['cmdRight',[1,0,0]],['climb',[0,0,1]],['descend',[0,0,-1]]]){
 const button=$(id);
 button.addEventListener('pointerdown',event=>{if(button.disabled)return;event.preventDefault();button.setPointerCapture(event.pointerId);heldPointers.set(event.pointerId,vector);});
 for(const type of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(type,event=>heldPointers.delete(event.pointerId));
}
$('cmdCenter').onclick=()=>{if(running)requestAction('pilot');};
$('returnHome').onclick=()=>{if(running)requestAction('return');};
$('demo').onclick=()=>{autoDemo=true;requestAction('start');};
window.addEventListener('blur',clearControls);
document.addEventListener('visibilitychange',()=>{if(document.hidden)clearControls();});
$('start').onclick=()=>{autoDemo=false;requestAction('start');};$('reset').onclick=()=>requestAction('reset');
$('pause').onclick=()=>{clearControls();paused=!paused;update();};
window.addEventListener('keydown',event=>{
 const target=document.activeElement;
 if($('visionDialog').open||target.matches('input,select,textarea,[role="tab"]'))return;
 if(event.code==='Space'){event.preventDefault();if(!event.repeat)$('pause').click();return;}
 if(event.code==='KeyR'){event.preventDefault();if(!event.repeat&&running)$('returnHome').click();return;}
 if(keyControls[event.code]&&running&&!paused){event.preventDefault();heldKeys.add(event.code);}
});
window.addEventListener('keyup',event=>heldKeys.delete(event.code));
async function tick(){
 if(busy || (!pendingAction && (!running || paused)))return;
 busy=true;const action=pendingAction||'step';pendingAction=null;
 try{
  const response=await fetch('/api/simulation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session,action,view:$('visionStage').value,manual:!autoDemo,flight:{altitude:+$('altitude').value,distance:+$('outbound').value,return_yaw:+$('yaw').value,tilt:+$('tilt').value},settings:{gps:+$('gps').value,wind:+$('wind').value,speed:+$('speed').value,vision:$('vision').checked,occluded:$('occluded').checked,manual_input:manualInput()}})});
  if(!response.ok)throw Error('Simulator request failed ('+response.status+'). Restart the server if needed.');
  const result=await response.json();
  if(['start','reset'].includes(action))trail=[];
  observation=result.observation;
  state={...result,lock:!!($('vision').checked && !$('occluded').checked && observation?.metric_valid)};
  if(action==='start')running=true;
  if(['READY','LANDED'].includes(state.phase)){running=false;clearControls();}
  if(trail.length===0||Math.hypot(...state.p.map((v,i)=>v-trail[trail.length-1][i]))>.1){trail.push([...state.p]);if(trail.length>1600)trail.shift();}
  const img=new Image();img.src='data:image/jpeg;base64,'+result.camera;await img.decode();frameImage=img;networkError='';
 }catch(error){networkError=error.message;running=false;clearControls();}finally{busy=false;update();}
}
$('visionStageLarge').innerHTML=$('visionStage').innerHTML;
function selectVision(value){
 $('visionStage').value=value;$('visionStageLarge').value=value;
 if(!pendingAction)pendingAction='view';
 tick();
}
$('visionStage').onchange=()=>selectVision($('visionStage').value);
$('visionStageLarge').onchange=()=>selectVision($('visionStageLarge').value);
$('expandVision').onclick=()=>{clearControls();$('visionStageLarge').value=$('visionStage').value;$('visionDialog').showModal();};
$('closeVision').onclick=()=>$('visionDialog').close();
$('visionPause').onclick=()=>$('pause').click();
const panelTabs=[...document.querySelectorAll('[role="tab"]')];
function selectPanel(tab){
 for(const item of panelTabs){const selected=item===tab;item.setAttribute('aria-selected',String(selected));item.tabIndex=selected?0:-1;$(item.getAttribute('aria-controls')).hidden=!selected;}
}
for(const tab of panelTabs){
 tab.addEventListener('click',()=>selectPanel(tab));
 tab.addEventListener('keydown',event=>{
  const index=panelTabs.indexOf(tab);
  let next=null;
  if(event.key==='ArrowRight')next=panelTabs[(index+1)%panelTabs.length];
  if(event.key==='ArrowLeft')next=panelTabs[(index+panelTabs.length-1)%panelTabs.length];
  if(event.key==='Home')next=panelTabs[0];
  if(event.key==='End')next=panelTabs[panelTabs.length-1];
  if(next){event.preventDefault();selectPanel(next);next.focus();}
 });
}
setInterval(tick,100);
let drag=null;world.onpointerdown=e=>{drag=[e.clientX,e.clientY];world.setPointerCapture(e.pointerId);};world.onpointerup=()=>drag=null;world.onpointercancel=()=>drag=null;world.onpointermove=e=>{if(drag){orbit-=(e.clientX-drag[0])*.008;elevation=clamp(elevation+(e.clientY-drag[1])*.005,.18,1.35);drag=[e.clientX,e.clientY];}};
world.addEventListener('wheel',e=>{e.preventDefault();zoom=clamp(zoom*Math.exp(-e.deltaY*.001),.08,2.8);},{passive:false});
function updateCommands(){
 const command=state.command, stopped=paused||!!networkError;
 const source=command?.source||'IDLE';
 $('commandPanel').dataset.source=source;
 const labels={IDLE:'IDLE',VISION:'VISION → simulator guidance',GPS:'GPS → simulated guidance','FLIGHT PLAN':'FLIGHT PLAN → simulated guidance',PILOT:'PILOT → manual flight'};
 $('commandSource').textContent=stopped?(paused?'PAUSED · last setpoint below':'DISCONNECTED · last setpoint below'):labels[source]||source;
 const [right,down]=command?.camera_horizontal_velocity_m_s||[0,0];
 const directions=[['Up',Math.max(0,-down)],['Down',Math.max(0,down)],['Left',Math.max(0,-right)],['Right',Math.max(0,right)]];
 for(const [direction,speed] of directions){$('cmd'+direction).classList.toggle('active',!stopped&&speed>.02);$(direction.toLowerCase()+'Speed').textContent=speed.toFixed(2)+' m/s';}
 $('cmdCenter').textContent=stopped?'PAUSED':'BRAKE';
 const velocity=command?.world_velocity_m_s||[0,0,0];
 $('verticalSource').textContent=command?.vertical_source==='PILOT'?'Vertical · pilot':'Vertical · simulator';
 $('verticalCommand').textContent=(command?.vertical||'HOLD')+' · '+Math.abs(velocity[2]).toFixed(2)+' m/s';
 $('worldCommand').textContent=velocity.slice(0,2).map(v=>v.toFixed(2)).join(' / ')+' m/s';
 $('trackerPixelOffset').textContent=observation?.valid?'Image Δx / Δy: '+observation.offset_px.map(v=>v.toFixed(1)).join(' / ')+' px':'No valid observation';
 $('trackerGroundOffset').textContent=observation?.metric_valid?'World Δx / Δy: '+observation.ground_offset_m.map(v=>v.toFixed(3)).join(' / ')+' m':'No valid metric offset';
 $('commandExplanation').textContent=source==='PILOT'?'Arrows follow the camera, not the orbit view. W/S changes height. R returns home.':source==='VISION'?'Tracker offset → simulator velocity. Vertical descent is simulator logic.':source==='GPS'?'GPS guides horizontally. Vision is not steering.':source==='FLIGHT PLAN'?'Takeoff/outbound motion follows the simulated plan.':'Tracker estimates position; simulator generates movement. Arrows light above 0.02 m/s.';
}
function update(){
 updateCommands();
 $('visionPause').textContent=paused?'Resume':'Pause';
 const diagnostic=state.vision_debug;
 if(diagnostic){$('visionDialogTitle').textContent='Vision · '+diagnostic.stage;$('visionCaption').textContent=`${diagnostic.mode} · ${diagnostic.features} features · ${diagnostic.matches} matches · ${diagnostic.inliers} inliers · ${diagnostic.alignment_status} · output: ${diagnostic.observation_status}${diagnostic.candidate_only?' · candidate only':''}`;}
 $('returnHome').disabled=!running;
 $('demo').disabled=running;
 const input=manualInput();
 for(const [id,axis,sign] of [['cmdUp',1,-1],['cmdDown',1,1],['cmdLeft',0,-1],['cmdRight',0,1],['climb',2,1],['descend',2,-1]]){$(id).disabled=!running||paused;$(id).classList.toggle('held',input[axis]*sign>0);}
 $('cmdCenter').disabled=!running||paused;
 $('flightSettings').disabled=running;
 for(const [id,unit] of [['altitude',' m'],['outbound',' m'],['yaw','°'],['tilt','°']])$(id+'Value').textContent=$(id).value+unit;
 $('attitude').textContent=(state.attitude_deg||[0,0,0]).map(x=>x.toFixed(1)+'°').join(' / ');
 $('badLocks').textContent=`${state.metrics?.false_locks||0} / ${state.metrics?.inaccurate_locks||0}`;
 for(const id of ['gps','wind','speed'])$(id+'Value').textContent=(+$(id).value).toFixed(1)+(id==='gps'?' m':id==='speed'?'×':'');
 $('phase').textContent=state.phase+(paused?' · PAUSED':'');$('pause').textContent=paused?'Resume':'Pause';
 $('height').textContent=state.p[2].toFixed(2)+' m';$('distance').textContent=Math.hypot(state.p[0],state.p[1]).toFixed(2)+' m';
 $('error').textContent=state.error===null?'—':state.error.toFixed(2)+' m';$('lock').textContent=!$('vision').checked?'Disabled':state.lock?'Locked':observation?.reason||'Mapping';
 const messages={READY:'Ready at home. Start a flight to remember the ground.',TAKEOFF:'Remembering home during ascent.',MANUAL:'Manual flight: hold arrows to move, W/S for height. Return home anytime.','RETURN CLIMB':'Climbing to return altitude before flying home.',OUTBOUND:'Flying away from the takeoff point.','GPS RETURN':'Returning toward the GPS estimate of home.',ACQUIRING:'Waiting for a stable visual observation. Descent held.','VISUAL APPROACH':'Correcting toward the remembered home point.',LANDED:'Flight complete. Change settings and start another flight.'};
 $('message').textContent=networkError||messages[state.phase];
 $('tracking').textContent=observation?`${observation.reason} · ${observation.quality.inliers||0} inliers · ${observation.latency_ms.toFixed(0)} ms · ${state.references||0} refs · map: ${state.mapping}`:`${state.mapping||'not_started'} · ${state.references||0} references`;
 $('simTime').textContent=state.time.toFixed(1)+' s';
 $('groundSize').textContent=(state.ground_width_m||256)+' m ground width';
}
function draw(){
 const rect=world.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);const w=rect.width,h=rect.height;
 if(world.width!==Math.round(w*dpr)||world.height!==Math.round(h*dpr)){world.width=Math.round(w*dpr);world.height=Math.round(h*dpr);}ctx.setTransform(dpr,0,0,dpr,0,0);
 ctx.fillStyle='#0b1521';ctx.fillRect(0,0,w,h);
 const distance=state.flight?.distance||+$('outbound').value, altitude=state.flight?.altitude||+$('altitude').value;
 const look=[distance*.32,distance*.18,altitude*.35], radius=Math.max(25,distance*1.7,altitude*3,state.distance*1.7||0);
 const eye=[look[0]+Math.cos(orbit)*radius*Math.cos(elevation)/zoom,look[1]+Math.sin(orbit)*radius*Math.cos(elevation)/zoom,look[2]+radius*Math.sin(elevation)/zoom];
 const norm=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);},cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
 const forward=norm(look.map((v,i)=>v-eye[i])),right=norm(cross(forward,[0,0,1])),up=cross(right,forward);
 function project(p){const v=p.map((x,i)=>x-eye[i]),z=dot(v,forward);return z>.2?[w/2+dot(v,right)*Math.min(w,h)*1.1/z,h*.54-dot(v,up)*Math.min(w,h)*1.1/z]:null;}
 function line(a,b,color,width=1){a=project(a);b=project(b);if(!a||!b)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
 function ring(p,r,color,width=1){for(let i=0;i<40;i++){let a=i*Math.PI/20,b=(i+1)*Math.PI/20;line([p[0]+r*Math.cos(a),p[1]+r*Math.sin(a),p[2]],[p[0]+r*Math.cos(b),p[1]+r*Math.sin(b),p[2]],color,width);}}
 const bound=(state.ground_width_m||256)/2, spacing=Math.max(8,Math.round(bound/128)*8);
 for(let i=-bound;i<=bound;i+=spacing){line([i,-bound,0],[i,bound,0],'#192c39');line([-bound,i,0],[bound,i,0],'#192c39');}
 for(let i=-20;i<=25;i++){line([i,-20,0],[i,25,0],'#203342');line([-20,i,0],[25,i,0],'#203342');}
 const boundary=[[-bound,-bound,0],[bound,-bound,0],[bound,bound,0],[-bound,bound,0]];
 boundary.forEach((p,i)=>line(p,boundary[(i+1)%4],'#426078',2));
 ring(HOME,1,'#5fd9ba',2);line([-1.3,0,.02],[1.3,0,.02],'#5fd9ba',2);line([0,-1.3,.02],[0,1.3,.02],'#5fd9ba',2);
 const bias=+$('gps').value;ring([bias,bias*.3,.03],.55,'#edb56b',2);
 for(let i=1;i<trail.length;i++)line(trail[i-1],trail[i],'#427782',1.5);
 const p=state.p;ring([p[0],p[1],.02],.45,'#3a5363',3);line([p[0],p[1],0],p,'#3c6274');
 const corners=(state.camera_footprint||[]).map(([x,y])=>[x,y,.04]);
 corners.forEach((c,i)=>{line(p,c,state.lock?'#326f65':'#2b4356');line(c,corners[(i+1)%corners.length],state.lock?'#55bfa6':'#426078');});
 const body=state.body_to_world||[[1,0,0],[0,1,0],[0,0,1]];
 const local=(x,y)=>[0,1,2].map(i=>p[i]+body[i][0]*x+body[i][1]*y);
 function rotor(x,y){for(let i=0;i<32;i++){const a=i*Math.PI/16,b=(i+1)*Math.PI/16;line(local(x+.32*Math.cos(a),y+.32*Math.sin(a)),local(x+.32*Math.cos(b),y+.32*Math.sin(b)),x>0?'#65d7bd':'#df936d',3);}}
 line(local(-.55,-.55),local(.55,.55),'#d8e7ef',7);line(local(-.55,.55),local(.55,-.55),'#d8e7ef',7);
 for(const x of [-.55,.55])for(const y of [-.55,.55])rotor(x,y);
 line(local(-.22,0),local(.27,0),'#f1f5fa',11);
 for(const [pos,label,color] of [[HOME,'HOME · remembered ground','#7de2cf'],[[bias,bias*.3,0],'GPS home estimate','#edb56b']]){const q=project(pos);if(q){ctx.fillStyle=color;ctx.font='12px system-ui';ctx.fillText(label,q[0]+10,q[1]+20);}}
 drawCamera();update();
}
function drawCamera(){
 const width=frameImage?.naturalWidth||640,height=frameImage?.naturalHeight||480;
 if(camera.width!==width||camera.height!==height){camera.width=width;camera.height=height;}
 cam.fillStyle='#253039';cam.fillRect(0,0,width,height);
 if(frameImage)cam.drawImage(frameImage,0,0,width,height);
 if($('visionDialog').open){
  const large=$('visionLarge');if(large.width!==width||large.height!==height){large.width=width;large.height=height;}
  const largeContext=large.getContext('2d');largeContext.fillStyle='#253039';largeContext.fillRect(0,0,width,height);
  if(frameImage)largeContext.drawImage(frameImage,0,0,width,height);
 }
}
function animate(){draw();requestAnimationFrame(animate);}
update();tick();requestAnimationFrame(animate);
