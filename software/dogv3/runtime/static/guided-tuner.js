/* Fixed gait trials. Loads and recordings never send ARM or drive intent. */
(() => {
  'use strict';
  const root=document.getElementById('guided-tuner');
  const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  let data=null,robot=null,selected='reference',busy=false,error='',signature='',revision=0,lastPoll=0;
  const pack=()=>data?.session?.method==='test_pack'?data.session:null;
  const find=id=>[data?.session?.reference,...(data?.session?.candidates||[])].find(c=>c?.id===id)
    ||data?.catalog?.find(c=>c.id===id)||{id:'reference',name:data?.reference_name||'Original trot',why:'Your original trot, kept exactly as saved.'};
  const group=()=>find(selected).seed?.gait_type||(selected.startsWith('deliberate')||selected.startsWith('weight-shift')?'crawl':'trot');
  const bestId=()=>pack()?.best_by_gait?.[group()]||(group()==='crawl'?'deliberate-low':'reference');
  const body=extra=>({session_id:data.session.id,...extra});
  const button=(text,attrs='',primary=false)=>`<button type="button" ${attrs} class="${primary?'gt-primary':''}">${text}</button>`;
  async function request(path,body){
    const r=await fetch('/api/tuner'+path,{method:body?'POST':'GET',
      headers:{'Content-Type':'application/json',...(KEY?{'X-DogV3-Key':KEY}:{})},body:body?JSON.stringify(body):undefined});
    const d=await r.json();
    if(!r.ok)throw Error(typeof d.detail==='string'?d.detail:'Check the entered values.');
    return d;
  }
  function options(){
    let html='<option value="reference">Original trot — turn-good-trot</option>';
    for(const family of [...new Set((data.catalog||[]).map(c=>c.family))]){
      html+=`<optgroup label="${esc(family)}">`;
      for(const c of data.catalog.filter(c=>c.family===family)){
        const failed=find(c.id).screen?.passed===false;
        html+=`<option value="${esc(c.id)}" ${failed?'disabled':''}>${esc(c.name)}${failed?' — blocked by checks':''}</option>`;
      }
      html+='</optgroup>';
    }
    return html;
  }
  function render(force=false){
    if(!data)return;
    const s=pack(),r=data.recording;
    const key=JSON.stringify([s,data.error,data.context_changed,r?.id,selected]);
    if(!force&&key===signature){update();return;}
    signature=key;
    const c=find(selected),best=find(bestId());
    const mode=data.mode==='physical'?'Physical test':data.mode==='simulation'?'Physics practice':'Local preview — no hardware';
    let h=`<div class="gt-heading"><h2>Test a gait</h2><span class="gt-mode">${mode}</span></div>
      <p class="gt-muted">Pick a gait. Load it. Drive. Rate it.</p>
      <div id="gt-error" class="gt-error" role="alert"></div><p id="gt-busy" class="gt-muted" role="status" hidden>Checking trajectories and preparing your test…</p>`;
    if(data.error){root.innerHTML=h+`<p class="gt-error">${esc(data.error)}</p>`;return;}
    if(data.session?.pending_recording&&!r)h+='<p class="gt-warning">An earlier recording was interrupted.</p>'+button('Discard unfinished recording','id="gt-discard"');
    if(data.context_changed&&s)h+='<p class="gt-warning">Configuration changed. Start a fresh test session under Details.</p>';
    h+=`<label for="gt-pick" class="gt-label">Gait to try</label><select id="gt-pick">${options()}</select>
      <p class="gt-description">${esc(c.why)}</p>`;
    if(c.screen){
      h+=c.screen.passed?'<p class="gt-pass">Trajectory checks passed · physical result unproven</p>':
        `<p class="gt-warning">${selected==='reference'?'Original retained for comparison. ':''}${esc(c.screen.blockers.join(' '))}</p>`;
    }
    if(!r){
      h+=`<div class="gt-actions">${button('Load & record','id="gt-load"',true)}${button(best.status==='best'?'Try best '+group():'Try reference','id="gt-best"')}</div>
        <p id="gt-load-state" class="gt-muted"></p>`;
      const last=s?.trials.at(-1);
      if(last){
        h+=`<p class="${last.valid&&!last.issues.length?'gt-pass':'gt-warning'}" role="status">Saved: ${esc(find(last.candidate_id).name)} — ${esc(last.outcome)}.`;
        if(!last.valid)h+=' Not counted: '+esc(last.invalid_reasons.join(' '));
        else if(last.issues.length)h+=' Reported problems prevent promotion.';
        else if(last.candidate_id===s.best_by_gait?.[find(last.candidate_id).seed.gait_type]&&last.compared_with!==last.candidate_id)h+=' Confirmed best in this gait group.';
        else if(last.outcome==='better')h+=' Repeat against the best to confirm.';
        h+='</p>';
      }
    }else{
      h+=`<div class="gt-record"><strong id="gt-timer">Recording</strong><progress id="gt-progress" max="1" value="0"></progress>
        <p id="gt-drive-hint" class="gt-muted">ARM with the normal controls, then hold W or push the left stick fully forward.</p>
        <div id="gt-faults"></div><p class="gt-label">${r.candidate_id===bestId()?'If this reference ran cleanly, choose Same.':'Compared with '+esc(best.name)}</p>
        <div class="gt-ratings">${button('Better','data-rating="better"',true)}${button('Same','data-rating="same"')}${button('Worse','data-rating="worse"')}${button('Unclear','data-rating="unclear"')}</div>
        <details><summary>Problems or notes (optional)</summary>
          <div class="gt-issues">${[['dragging','Feet scuffed'],['slipping','Slipped'],['rocking','Rocked'],['fell','Fell / caught'],['hot_or_power_issue','Heat / power concern']].map(([v,l])=>`<label><input type="checkbox" name="gt-issue" value="${v}">${l}</label>`).join('')}</div>
          <textarea id="gt-note" maxlength="1000" aria-label="Test notes" placeholder="What did you notice?"></textarea>
        </details>${button('Discard recording','id="gt-discard"')}</div>`;
    }
    const baseline=s?.trials.some(t=>t.candidate_id===bestId()&&t.valid&&!t.issues.length&&['same','better'].includes(t.outcome));
    if(selected!==bestId()&&!baseline)h+=`<p class="gt-warning">First record ${esc(best.name)} once as a clean reference.</p>`;
    h+=`<p class="gt-muted">${best.status==='best'?'Best':'Reference'} ${group()}: <strong>${esc(best.name)}</strong>. Two clean better runs confirm a replacement.</p>
      <details id="gt-details"><summary>Details, results & session</summary>
      <p class="gt-muted">Start with the lower-lift version of each style. Try extra clearance only for scuffing. Judge trot against trot and crawl against crawl on the same surface. Support or spot the robot with E-stop ready.</p>`;
    if(c.seed)h+=`<p class="gt-muted">${esc(c.seed.gait_type)} · ${c.seed.cycle_period} s cycle · ${c.seed.step_height} mm lift · ${Math.round(c.seed.duty_factor*100)}% stance</p>
      <p class="gt-muted">Planned full-input speed: ${c.screen.metrics?.nominal_full_stick_mms??'—'} mm/s. Checks cover steady straight motion, not turns, starts/stops, traction or balance.</p>
      <details><summary>Exact settings and check numbers</summary><pre>${esc(JSON.stringify({settings:c.seed,checks:c.screen},null,2))}</pre></details>`;
    if(s)h+=`<div class="gt-history">${s.trials.slice().reverse().map(t=>`<p>${esc(find(t.candidate_id).name)}: <strong>${esc(t.outcome)}</strong>${t.valid?'':' (not counted)'}${t.note?' · '+esc(t.note):''}</p>`).join('')||'<p>No results yet.</p>'}</div>`;
    h+=`${button('Download all history','id="gt-history"')}
      <label class="gt-label" for="gt-surface">Surface / support setup for next session</label><input id="gt-surface" maxlength="160" value="${esc(s?.surface||'Same surface throughout this session; unspecified')}">
      ${button('Start fresh session','id="gt-new"')}
      <label class="gt-label" for="gt-export-name">Save confirmed best ${group()} under a new profile name</label><input id="gt-export-name" maxlength="80" placeholder="e.g. tested-floor-trot">
      ${button('Save named profile','id="gt-export"')}</details>`;
    root.innerHTML=h;
    root.querySelector('#gt-pick').value=selected;
    update();
  }
  function update(){
    if(!data)return;
    const err=root.querySelector('#gt-error');if(err)err.textContent=error;
    const b=root.querySelector('#gt-busy');if(b)b.hidden=!busy;
    const r=data.recording,armed=!!robot?.telemetry?.armed;
    const pending=!!data.session?.pending_recording&&!r;
    root.querySelectorAll('button').forEach(x=>x.disabled=busy);
    const pick=root.querySelector('#gt-pick');if(pick)pick.disabled=busy||!!r||pending;
    for(const id of ['gt-load','gt-best','gt-new']){
      const el=root.querySelector('#'+id);if(el)el.disabled=busy||armed||!!r||pending||!!robot?.auto_mode;
    }
    const ld=root.querySelector('#gt-load-state');
    if(ld)ld.textContent=armed?'Disarm before loading the next gait.':'Loading starts a recording. ARM and driving stay manual.';
    const ex=root.querySelector('#gt-export');if(ex)ex.disabled=busy||!!r||find(bestId()).status!=='best';
    if(r){
      const c=find(r.candidate_id),goal=Math.ceil(Math.max(3,3*c.seed.cycle_period)*10)/10;
      const seconds=r.longest_steady_seconds||0,stopped=Math.abs(robot?.telemetry?.cmd?.vy||0)<.03;
      const ready=seconds>=goal&&!r.faults.length;
      root.querySelector('#gt-timer').textContent=ready?'Steady run captured':`${seconds.toFixed(1)} / ${goal.toFixed(1)} s steady forward`;
      root.querySelector('#gt-progress').value=Math.min(1,seconds/goal);
      root.querySelector('#gt-drive-hint').textContent=!armed?(ready?'Choose your result below.':'ARM, then hold W or push the left stick fully forward.'):
        ready?'Release drive, then rate the gait.':'Hold W or full forward stick. Keep turning and height trim at zero.';
      root.querySelector('#gt-faults').innerHTML=r.faults.map(f=>`<p class="gt-warning">${esc(f)}</p>`).join('');
      root.querySelectorAll('[data-rating]').forEach(x=>x.disabled=busy||!stopped||(x.dataset.rating==='better'&&r.candidate_id===bestId()));
    }
  }
  async function action(fn){
    if(busy)return;busy=true;revision++;error='';update();
    try{await fn();render();}catch(e){error=e.message;}finally{busy=false;update();}
  }
  async function loadTest(){
    if(!pack())data=await request('/begin-pack',{});
    if(data.context_changed)throw Error('Start a fresh test session under Details.');
    const id=selected,tag=data.session.id+'/'+id;
    data=await request('/prepare',body({candidate_id:id}));
    // Wait for explicit control-loop acknowledgement, never a guessed delay.
    for(let i=0;i<50;i++){
      data=await request('');
      if(data.load?.candidate_id===tag&&data.load.status==='refused')throw Error(data.load.reason);
      if(data.load?.candidate_id===tag&&data.load.status==='loaded'){
        data=await request('/record',body({candidate_id:id}));return;
      }
      await new Promise(resolve=>setTimeout(resolve,50));
    }
    throw Error('The control loop did not acknowledge the load. Disarm and try again.');
  }
  root.addEventListener('change',e=>{if(e.target.id==='gt-pick'){selected=e.target.value;render();}});
  root.addEventListener('click',e=>{
    const b=e.target.closest('button');if(!b||b.disabled)return;
    if(b.id==='gt-load')action(loadTest);
    else if(b.id==='gt-best'){selected=bestId();render();action(loadTest);}
    else if(b.dataset.rating)action(async()=>{data=await request('/result',body({candidate_id:data.recording.candidate_id,
      recording_id:data.recording.id,outcome:b.dataset.rating,
      issues:[...root.querySelectorAll('[name=gt-issue]:checked')].map(x=>x.value),note:root.querySelector('#gt-note')?.value||''}));});
    else if(b.id==='gt-discard')action(async()=>{data=await request('/discard',body({}));});
    else if(b.id==='gt-new')action(async()=>{data=await request('/begin-pack',{surface:root.querySelector('#gt-surface').value});});
    else if(b.id==='gt-export')action(async()=>{data=await request('/export',body({name:root.querySelector('#gt-export-name').value,gait_type:group()}));});
    else if(b.id==='gt-history')action(async()=>{
      const h=await request('/history'),url=URL.createObjectURL(new Blob([JSON.stringify(h,null,2)],{type:'application/json'}));
      const a=document.createElement('a');a.href=url;a.download='dogv3-'+data.mode+'-trials.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    });
  });
  async function poll(){
    if(busy)return;const epoch=revision;
    try{const result=await request('');if(epoch!==revision||busy)return;data=result;
      if(data.recording&&pack())selected=data.recording.candidate_id;render();}
    catch(e){if(epoch!==revision||busy)return;error=e.message;if(!data)root.innerHTML='<p class="gt-error">'+esc(error)+'</p>';update();}
  }
  window.GuidedTunerUI={observe(s){robot=s;update();if(!busy&&Date.now()-lastPoll>1500){lastPoll=Date.now();poll();}}};
  robot=typeof latest==='undefined'?null:latest;poll();
})();
