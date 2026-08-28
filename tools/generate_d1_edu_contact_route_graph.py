#!/usr/bin/env python3
"""Generate a 30-episode route graph using the corrected contact semantics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from d1_edu_sb3_env import D1DecisionEnv
from d1_edu_sb3_policy import SB3DecisionPolicy
from run_d1_edu_decision_layer_viewer import EpisodeConfig, run_episode


def _compact_trace(trace: tuple[dict[str, Any], ...], stride: int = 4) -> list[dict[str, Any]]:
    if not trace:
        return []
    selected = list(trace[::stride])
    if selected[-1] is not trace[-1]:
        selected.append(trace[-1])
    return selected


def _json_for_script(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=True)


def _html(payload: dict[str, Any]) -> str:
    data = _json_for_script(payload)
    # Keep the HTML readable and independent of external chart libraries.
    return r'''<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>D1 Edu · Nav2 global path + PPO + MPPI routes</title>
<style>
:root{color-scheme:dark;--bg:#11161d;--panel:#18212b;--plot:#0b1118;--grid:#2b3947;--text:#ecf2f8;--muted:#9aabba;--blue:#69b8ff;--orange:#ff9c52;--red:#ff5d70;--green:#57d58a;--yellow:#ffd166;--purple:#b28cff;--cyan:#51d1c1}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(135deg,#10151c,#182430);color:var(--text);font:15px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Display",Segoe UI,sans-serif}main{max-width:1450px;margin:0 auto;padding:28px 32px 60px}h1{font-size:32px;margin:0 0 6px;letter-spacing:.01em}.subtitle{color:var(--muted);font-size:16px;margin-bottom:22px}.cards{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:18px}.card{background:#202c37;border:1px solid #324454;border-radius:11px;padding:11px 15px;min-width:126px}.card b{display:block;font-size:21px}.card span{color:var(--muted);font-size:12px}.toolbar{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin:14px 0}.toolbar label{color:var(--muted)}select{background:#202c37;color:var(--text);border:1px solid #587184;border-radius:8px;padding:9px 12px;font-size:15px;min-width:260px}.legend{display:flex;gap:15px;flex-wrap:wrap;color:var(--muted);font-size:13px}.key{display:inline-flex;align-items:center;gap:6px}.swatch{width:24px;height:4px;border-radius:4px;display:inline-block}.dash{height:0;border-top:3px dashed var(--purple)}.panel{background:rgba(17,24,31,.82);border:1px solid #334554;border-radius:16px;padding:18px;margin-top:18px;box-shadow:0 14px 50px #07101888}.panel h2{font-size:17px;margin:0 0 8px}.panel p{color:var(--muted);margin:0 0 12px}.overview{display:grid;grid-template-columns:repeat(10,minmax(0,1fr));gap:6px}.tile{height:30px;border:1px solid #30404e;border-radius:5px;cursor:pointer;position:relative;overflow:hidden;background:#24313c}.tile:hover{border-color:#dbe9f4}.tile.selected{outline:2px solid #e9f4ff;outline-offset:1px}.tile .bar{height:100%;width:100%}.tile span{position:absolute;inset:0;display:grid;place-items:center;font-size:11px;color:#081018;font-weight:700;text-shadow:0 1px #ffffff77}.route-wrap{height:620px;position:relative}.route-wrap svg{width:100%;height:100%;display:block;background:var(--plot);border-radius:11px;border:1px solid #30414f}.route-info{display:flex;gap:22px;flex-wrap:wrap;color:var(--muted);font-size:14px;margin-top:10px}.route-info b{color:var(--text)}.note{color:var(--muted);font-size:13px;margin-top:10px}.status-success{color:var(--green)}.status-contact{color:var(--red)}.status-geometry{color:var(--orange)}.status-timeout{color:var(--yellow)}.status-proxy{color:#f5c46a}.tooltip{position:fixed;pointer-events:none;background:#0b1118eF;border:1px solid #628099;border-radius:7px;padding:5px 8px;font-size:12px;display:none;z-index:4}.footer{color:var(--muted);font-size:12px;margin-top:22px}@media(max-width:800px){main{padding:18px}.overview{grid-template-columns:repeat(5,1fr)}.route-wrap{height:470px}}
</style>
</head>
<body><main>
<h1>智元 D1 Edu · Nav2 全域路徑 + PPO + MPPI</h1>
<div class="subtitle" id="subtitle"></div>
<div class="cards" id="cards"></div>
<div class="toolbar"><label for="trial">查看路線</label><select id="trial"></select><span id="status"></span></div>
<div class="legend">
 <span class="key"><i class="swatch" style="background:var(--blue)"></i>CRUISE</span>
 <span class="key"><i class="swatch" style="background:var(--orange)"></i>AVOID</span>
 <span class="key"><i class="swatch" style="background:var(--yellow)"></i>WAIT_YIELD</span>
 <span class="key"><i class="swatch" style="background:var(--purple)"></i>Nav2 全域路徑</span>
 <span class="key"><i class="swatch" style="background:var(--green)"></i>行人代理</span>
 <span class="key"><i class="swatch dash"></i>障礙物實際軌跡</span>
 <span class="key"><i class="swatch" style="background:var(--red)"></i>MuJoCo physical contact</span>
 <span class="key"><i class="swatch" style="background:var(--yellow)"></i>near miss / proxy-only / timeout</span>
</div>
<section class="panel"><h2>trial overview</h2><p>點選色塊切換路線；紫色虛線是 Nav2-style 全域路徑，實線是 PPO + MPPI 的實際機器人路徑。</p><div class="overview" id="overview"></div></section>
<section class="panel"><h2 id="plot-title">selected trajectory</h2><div class="route-wrap"><svg id="plot" viewBox="0 0 1000 620" role="img" aria-label="D1 route plot"></svg></div><div class="route-info" id="route-info"></div><div class="note">虛線是障礙物中心的實際移動軌跡；紅色只代表 MuJoCo 報告 robot–obstacle contact。橘色圈是精確矩形幾何重疊，黃色則是接近或只有保守外接圓 proxy 告警。</div></section>
<div class="footer" id="footer"></div>
</main><div class="tooltip" id="tip"></div>
<script>
const DATA = __DATA__;
const COLORS={CRUISE:'#69b8ff',AVOID:'#ff9c52',WAIT_YIELD:'#ffd166',success:'#57d58a',collision:'#ff5d70',geometry:'#ff9c52',timeout:'#ffd166',proxy:'#f5c46a',obstacle:['#b28cff','#51d1c1','#ef83c0','#8fd0ff'],person:'#57d58a'};
const fmt=(v,n=3)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'—';
const esc=s=>String(s).replace(/[&<>"']/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]));
function outcome(r){if(r.physical_contact||r.collision)return ['collision','MuJoCo contact'];if(r.geometric_collision)return ['geometry','geometric overlap'];if(r.success)return ['success','success'];if(r.proxy_only)return ['proxy','proxy-only'];return ['timeout','timeout / goal failure'];}
function routeColor(r){const [c]=outcome(r);return COLORS[c]||COLORS.timeout;}
function renderCards(){const s=DATA.summary;const items=[['trials',s.trials],['success',`${s.successful_trials} (${fmt(s.success_rate*100,1)}%)`],['physical contact',`${s.physical_contact_count} (${fmt(s.physical_contact_rate*100,1)}%)`],['geometry overlap',`${s.geometric_collision_count} (${fmt(s.geometric_collision_rate*100,1)}%)`],['timeout',`${s.timeout_count} (${fmt(s.timeout_rate*100,1)}%)`],['WAIT argmax',`${fmt(s.wait_argmax_rate*100,1)}%`],['invalid WAIT',`${fmt(s.invalid_wait_rate*100,1)}%`],['near-goal timeout',`${s.near_goal_timeout_count||0}`],['real deadlock',`${s.real_deadlock_count||0}`]];document.getElementById('cards').innerHTML=items.map(x=>`<div class="card"><b>${esc(x[1])}</b><span>${esc(x[0])}</span></div>`).join('');document.getElementById('subtitle').textContent=`${s.trials} routes · ${fmt(s.route_length,1)} m route · physical-contact evaluation · PPO model ${DATA.model}`;document.getElementById('footer').textContent=`model: Nav2 global path + PPO + MPPI · profile: full_randomization · scenario: ${s.scenario_mode||'unknown'} · route: ${fmt(s.route_length,1)} m · duration: ${fmt(s.duration,1)} s`;}
function renderOverview(){const el=document.getElementById('overview');el.innerHTML=DATA.routes.map((r,i)=>`<div class="tile" data-i="${i}" title="trial ${i}: ${esc(outcome(r)[1])}"><div class="bar" style="background:${routeColor(r)}"></div><span>${i}</span></div>`).join('');el.querySelectorAll('.tile').forEach(t=>t.onclick=()=>select(Number(t.dataset.i)));}
function linePath(points,sx,sy){return points.map((p,i)=>(i?'L':'M')+sx(p[0]).toFixed(1)+','+sy(p[1]).toFixed(1)).join(' ');}
function pathSegments(trace){const out=[];let current=null;for(const row of trace){const mode=row.mode||'CRUISE';if(!current||current.mode!==mode){current={mode,points:[]};out.push(current);}current.points.push(row.robot);}return out.filter(s=>s.points.length>1);}
function renderPlot(index){const r=DATA.routes[index];const trace=r.trace||[];const svg=document.getElementById('plot');const width=1000,height=620;const pad={l:70,r:24,t:28,b:58};const robots=trace.map(x=>x.robot);const goal=(trace[0]&&trace[0].goal)||[r.goal_x||3.2,0];const all=robots.length?robots:[[0,0]];const xs=all.map(p=>p[0]).concat([goal[0]]);const ys=all.map(p=>p[1]).concat([goal[1]]);let xmin=Math.min(...xs)-.55,xmax=Math.max(...xs)+.55,ymin=Math.min(...ys)-1.0,ymax=Math.max(...ys)+1.0;if(xmax-xmin<2)xmax=xmin+2;if(ymax-ymin<2)ymax=ymin+2;const sx=x=>pad.l+(x-xmin)/(xmax-xmin)*(width-pad.l-pad.r);const sy=y=>height-pad.b-(y-ymin)/(ymax-ymin)*(height-pad.t-pad.b);let html=`<defs><clipPath id="clip"><rect x="${pad.l}" y="${pad.t}" width="${width-pad.l-pad.r}" height="${height-pad.t-pad.b}" rx="8"/></clipPath></defs>`;for(let i=0;i<=6;i++){const x=pad.l+i*(width-pad.l-pad.r)/6;const value=xmin+i*(xmax-xmin)/6;html+=`<line x1="${x}" y1="${pad.t}" x2="${x}" y2="${height-pad.b}" stroke="#2b3947"/><text x="${x}" y="${height-25}" fill="#9aabba" text-anchor="middle" font-size="12">${fmt(value,1)} m</text>`;}for(let i=0;i<=5;i++){const y=pad.t+i*(height-pad.t-pad.b)/5;const value=ymax-i*(ymax-ymin)/5;html+=`<line x1="${pad.l}" y1="${y}" x2="${width-pad.r}" y2="${y}" stroke="#2b3947"/><text x="${pad.l-12}" y="${y+4}" fill="#9aabba" text-anchor="end" font-size="12">${fmt(value,1)}</text>`;}html+=`<line x1="${sx(0)}" y1="${pad.t}" x2="${sx(0)}" y2="${height-pad.b}" stroke="#718293"/><line x1="${pad.l}" y1="${sy(0)}" x2="${width-pad.r}" y2="${sy(0)}" stroke="#718293"/>`;
 const first=trace[0]?trace[0].robot:[0,0],last=trace.length?trace[trace.length-1].robot:first;
 const globalPath=(trace[0]&&trace[0].global_path&&trace[0].global_path.length>1)?trace[0].global_path:[first,goal];
 html+=`<path d="${linePath(globalPath,sx,sy)}" fill="none" stroke="${COLORS.purple||'#b28cff'}" stroke-width="3" stroke-dasharray="10 8" opacity=".95" clip-path="url(#clip)"/>`;
 const obstacleCount=(trace[0]&&trace[0].obstacles||[]).length;for(let oi=0;oi<obstacleCount;oi++){const items=trace.map(row=>row.obstacles[oi]).filter(o=>o&&o.active);const pts=items.map(o=>[o.x,o.y]);const isPerson=items.some(o=>o.kind==='person');const isObject=items.some(o=>o.kind==='object');const color=isPerson?COLORS.person:COLORS.obstacle[oi%COLORS.obstacle.length];const label=isPerson?'行人':(isObject?'物體':'障礙物');const motion=items.length?items[0].motion_kind:'';if(pts.length>1){html+=`<path d="${linePath(pts,sx,sy)}" fill="none" stroke="${color}" stroke-width="3" stroke-dasharray="9 7" opacity=".9" clip-path="url(#clip)"/>`;const last=pts[pts.length-1];html+=`<circle cx="${sx(last[0])}" cy="${sy(last[1])}" r="${isPerson?9:7}" fill="${color}" opacity=".95"/><text x="${sx(last[0])+11}" y="${sy(last[1])-9}" fill="${color}" font-size="12">${label} ${oi+1} · ${motion}</text>`;}}
 for(const seg of pathSegments(trace)){html+=`<path d="${linePath(seg.points,sx,sy)}" fill="none" stroke="${COLORS[seg.mode]||COLORS.CRUISE}" stroke-width="5" stroke-linecap="round" stroke-linejoin="round" clip-path="url(#clip)"/>`;}
 html+=`<circle cx="${sx(first[0])}" cy="${sy(first[1])}" r="9" fill="#ecf2f8"/><text x="${sx(first[0])+13}" y="${sy(first[1])-10}" fill="#ecf2f8" font-size="13">start</text><circle cx="${sx(goal[0])}" cy="${sy(goal[1])}" r="9" fill="#d9e2eb"/><text x="${sx(goal[0])+13}" y="${sy(goal[1])-10}" fill="#ecf2f8" font-size="13">goal</text><circle cx="${sx(last[0])}" cy="${sy(last[1])}" r="9" fill="${routeColor(r)}"/><text x="${sx(last[0])+13}" y="${sy(last[1])+20}" fill="#ecf2f8" font-size="13">end</text>`;
 const lastTrace=trace.length?trace[trace.length-1]:{};const contactIndex=Number.isInteger(r.collision_obstacle_index)?r.collision_obstacle_index:(Number.isInteger(lastTrace.collision_obstacle_index)?lastTrace.collision_obstacle_index:-1);const contactObstacle=contactIndex>=0&&lastTrace.obstacles&&lastTrace.obstacles[contactIndex]&&lastTrace.obstacles[contactIndex].active?lastTrace.obstacles[contactIndex]:null;const contactPoint=r.contact_position||lastTrace.contact_position;if(r.physical_contact){html+=`<circle cx="${sx(last[0])}" cy="${sy(last[1])}" r="17" fill="none" stroke="${COLORS.collision}" stroke-width="3"/><text x="${sx(last[0])+22}" y="${sy(last[1])+4}" fill="${COLORS.collision}" font-size="12">CONTACT robot</text>`;if(contactObstacle){html+=`<line x1="${sx(last[0])}" y1="${sy(last[1])}" x2="${sx(contactObstacle.x)}" y2="${sy(contactObstacle.y)}" stroke="${COLORS.collision}" stroke-width="2" stroke-dasharray="5 4"/><circle cx="${sx(contactObstacle.x)}" cy="${sy(contactObstacle.y)}" r="15" fill="none" stroke="${COLORS.collision}" stroke-width="3"/><text x="${sx(contactObstacle.x)+18}" y="${sy(contactObstacle.y)-10}" fill="${COLORS.collision}" font-size="12">CONTACT obstacle ${contactIndex+1}</text>`;}if(Array.isArray(contactPoint)&&contactPoint.length>=2){html+=`<circle cx="${sx(contactPoint[0])}" cy="${sy(contactPoint[1])}" r="6" fill="${COLORS.collision}"/><text x="${sx(contactPoint[0])+10}" y="${sy(contactPoint[1])-8}" fill="${COLORS.collision}" font-size="12">contact point</text>`;}}}else if(r.geometric_collision){html+=`<circle cx="${sx(last[0])}" cy="${sy(last[1])}" r="17" fill="none" stroke="${COLORS.geometry}" stroke-width="3" stroke-dasharray="5 4"/>`;}
 html+=`<text x="${pad.l}" y="18" fill="#ecf2f8" font-size="15">trial ${index} · ${esc(outcome(r)[1])} · ${trace.length} samples</text><text x="${width-pad.r}" y="18" fill="#9aabba" text-anchor="end" font-size="13">x [m] / y [m]</text>`;svg.innerHTML=html;document.getElementById('plot-title').textContent=`trial ${index} · ${outcome(r)[1]} · selected trajectory`;const [kind,label]=outcome(r);document.getElementById('status').className='status-'+kind;document.getElementById('status').textContent=label;const m=r.decision_metrics||{};document.getElementById('route-info').innerHTML=`<span><b>min clearance</b> ${fmt(r.min_clearance)} m</span><span><b>proxy clearance</b> ${fmt(r.min_proxy_clearance)} m</span><span><b>final goal distance</b> ${fmt(r.distance_to_goal)} m</span><span><b>path</b> ${fmt(r.path_length)} m</span><span><b>WAIT argmax</b> ${fmt(m.wait_argmax_rate*100,1)}%</span><span><b>invalid WAIT</b> ${fmt(m.invalid_wait_rate*100,1)}%</span><span><b>wait→CRUISE</b> ${m.wait_returned_to_cruise_count||0}</span><span><b>real deadlock</b> ${m.real_deadlock?'yes':'no'}</span><span><b>final path error</b> ${fmt(lastTrace.path_lateral_error)} m</span><span><b>global progress</b> ${fmt(lastTrace.path_progress*100,1)}%</span><span><b>physical contact</b> ${r.physical_contact?'yes':'no'}</span><span><b>contact obstacle</b> ${contactIndex>=0?contactIndex+1:'—'}</span><span><b>contact geom</b> ${esc(r.contact_geom_name||lastTrace.contact_geom_name||'—')}</span><span><b>contact point</b> ${Array.isArray(contactPoint)?`(${fmt(contactPoint[0],3)}, ${fmt(contactPoint[1],3)}, ${fmt(contactPoint[2],3)})`:'—'}</span><span><b>contact dist</b> ${fmt(r.contact_distance??lastTrace.contact_distance,4)} m</span><span><b>geometric overlap</b> ${r.geometric_collision?'yes':'no'}</span>`;document.querySelectorAll('.tile').forEach((t,i)=>t.classList.toggle('selected',i===index));document.getElementById('trial').value=String(index);}
 function select(i){renderPlot(i)}
const selectEl=document.getElementById('trial');selectEl.innerHTML=DATA.routes.map((r,i)=>`<option value="${i}">trial ${i} · ${esc(outcome(r)[1])}</option>`).join('');selectEl.onchange=()=>select(Number(selectEl.value));renderCards();renderOverview();select(0);
</script></body></html>'''.replace('__DATA__', data)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--duration", type=float, default=80.0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument(
        "--profile",
        choices=("baseline", "sensor_noise", "full_randomization"),
        default="full_randomization",
    )
    parser.add_argument(
        "--scenario-mode",
        choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target", "wait_yield_curriculum", "long_route"),
        default="people_multi_target",
    )
    parser.add_argument("--mppi-samples", type=int, default=32)
    parser.add_argument("--mppi-horizon", type=int, default=30)
    args = parser.parse_args()

    policy = SB3DecisionPolicy(args.model, deterministic=True)
    routes: list[dict[str, Any]] = []
    for trial in range(args.trials):
        case_seed = args.seed + 10000 + trial
        env = D1DecisionEnv(
            profile=args.profile,
            scenario_mode=args.scenario_mode,
            seed=case_seed,
            duration=args.duration,
            route_length=args.route_length,
            mppi_samples=args.mppi_samples,
            mppi_horizon=args.mppi_horizon,
            cruise_mode="goal_hysteresis",
        )
        reset_options = (
            {"curriculum_index": trial}
            if args.scenario_mode in {
                "wait_yield_curriculum",
                "people_balanced_multi_target",
            }
            else None
        )
        _, info = env.reset(seed=case_seed, options=reset_options)
        config: EpisodeConfig = info["config"]
        episode_seed = int(info["episode_seed"])
        result = run_episode(
            config,
            episode_seed,
            decision_policy=policy,
            record_trace=True,
        )
        env.close()
        routes.append({
            "trial": trial,
            "seed": episode_seed,
            "curriculum_case": config.curriculum_case,
            "success": bool(result.success),
            "collision": bool(result.collision),
            "physical_contact": bool(result.physical_contact),
            "geometric_collision": bool(result.geometric_collision),
            "proxy_collision": bool(result.proxy_collision),
            "proxy_only": bool(result.proxy_only),
            "near_miss": bool(result.near_miss),
            "timeout": bool(result.timeout),
            "min_clearance": float(result.min_clearance),
            "min_proxy_clearance": float(result.min_proxy_clearance),
            "distance_to_goal": float(result.distance_to_goal),
            "path_length": float(result.path_length),
            "elapsed_time": float(result.elapsed_time),
            "failure_reason": result.failure_reason,
            "decision_metrics": result.decision_metrics,
            "collision_obstacle_index": result.collision_obstacle_index,
            "contact_geom_name": result.contact_geom_name,
            "contact_distance": float(result.contact_distance),
            "contact_position": result.contact_position,
            "goal_x": float(config.goal_x),
            "trace": _compact_trace(result.trace),
        })
        print(f"trial={trial:02d} status={routes[-1]['success'] and 'success' or result.failure_reason} contact={result.physical_contact} samples={len(result.trace)}")

    def count(key: str) -> int:
        return sum(bool(route[key]) for route in routes)

    summary = {
        "trials": len(routes),
        "successful_trials": count("success"),
        "success_rate": count("success") / len(routes) if routes else float("nan"),
        "collision_count": count("collision"),
        "physical_contact_count": count("physical_contact"),
        "physical_contact_rate": count("physical_contact") / len(routes) if routes else float("nan"),
        "geometric_collision_count": count("geometric_collision"),
        "geometric_collision_rate": count("geometric_collision") / len(routes) if routes else float("nan"),
        "proxy_only_count": count("proxy_only"),
        "near_miss_count": count("near_miss"),
        "timeout_count": count("timeout"),
        "timeout_rate": count("timeout") / len(routes) if routes else float("nan"),
        "mean_min_clearance": float(np.mean([r["min_clearance"] for r in routes])) if routes else float("nan"),
        "route_length": float(args.route_length),
        "duration": float(args.duration),
        "model": str(args.model),
        "scenario_mode": args.scenario_mode,
        "waitable_states": int(sum(
            route.get("decision_metrics", {}).get("waitable_states", 0)
            for route in routes
        )),
        "valid_wait_count": int(sum(
            route.get("decision_metrics", {}).get("valid_wait_count", 0)
            for route in routes
        )),
        "invalid_wait_count": int(sum(
            route.get("decision_metrics", {}).get("invalid_wait_count", 0)
            for route in routes
        )),
        "wait_argmax_rate": (
            float(sum(route.get("decision_metrics", {}).get("valid_wait_count", 0) for route in routes))
            / float(sum(route.get("decision_metrics", {}).get("waitable_states", 0) for route in routes))
            if sum(route.get("decision_metrics", {}).get("waitable_states", 0) for route in routes)
            else float("nan")
        ),
        "invalid_wait_rate": (
            float(sum(route.get("decision_metrics", {}).get("invalid_wait_count", 0) for route in routes))
            / float(sum(route.get("decision_metrics", {}).get("invalid_wait_count", 0) + route.get("decision_metrics", {}).get("valid_wait_count", 0) for route in routes))
            if sum(route.get("decision_metrics", {}).get("invalid_wait_count", 0) + route.get("decision_metrics", {}).get("valid_wait_count", 0) for route in routes)
            else 0.0
        ),
        "near_goal_timeout_count": int(sum(
            bool(route.get("decision_metrics", {}).get("near_goal_timeout", False))
            for route in routes
        )),
        "real_deadlock_count": int(sum(
            bool(route.get("decision_metrics", {}).get("real_deadlock", False))
            for route in routes
        )),
    }
    payload = {"model": Path(args.model).name, "summary": summary, "routes": routes}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep generated route graphs valid in browsers. The contact branch has
    # two nested conditionals; older generated artifacts contained one extra
    # closing brace before the geometric-overlap ``else``.
    html = _html(payload).replace(
        ";}}}else if(r.geometric_collision)",
        ";}}else if(r.geometric_collision)",
    )
    output.write_text(html, encoding="utf-8")
    json_path = output.with_suffix(".json")
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True), encoding="utf-8")
    print(f"html={output}")
    print(f"json={json_path}")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
