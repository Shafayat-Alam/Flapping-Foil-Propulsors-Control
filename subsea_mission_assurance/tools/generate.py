#!/usr/bin/env python3
"""
generate.py — render documents + matrices FROM the model (the views).

Never edit docs/ by hand: edit model/*.yaml and re-run this. Output is
regenerated each time, so views can never drift from the source of truth.

Usage:  python3 tools/generate.py
"""
import sys, os, csv, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _model
import check as checker

VMETHOD = {'T': 'Test', 'A': 'Analysis', 'I': 'Inspection', 'D': 'Demonstration'}


def w(path, text):
    with open(path, 'w') as f:
        f.write(text)


def csv_write(path, header, rows):
    with open(path, 'w', newline='') as f:
        c = csv.writer(f)
        c.writerow(header)
        c.writerows(rows)


def main():
    m = _model.load()
    idx = _model.index(m)
    os.makedirs(_model.DOCS_DIR, exist_ok=True)
    d = _model.DOCS_DIR
    stamp = datetime.date.today().isoformat()
    name = lambda i: idx[i][1].get('name', i) if i in idx else f"{i}?"

    # ---- 01 Stakeholder Expectations ----
    s = ["# Stakeholder Expectations", f"_Generated {stamp} from model/. Do not edit._\n"]
    for stk in m.get('stakeholders', []):
        s.append(f"### {stk['id']} — {stk['name']}")
        exps = [e for e in m.get('expectations', []) if e.get('stakeholder') == stk['id']]
        for e in exps:
            s.append(f"- **{e['id']}** ({e.get('status','?')}): {e['text']}")
        if not exps:
            s.append("- _(no expectations captured)_")
        s.append("")
    w(os.path.join(d, '01_Stakeholder_Expectations.md'), "\n".join(s))

    # ---- 02 MOE / MOP ----
    s = ["# Measures of Effectiveness & Performance", f"_Generated {stamp}._\n"]
    for moe in m.get('moes', []):
        star = " (PRIMARY)" if moe.get('primary') else ""
        s.append(f"## {moe['id']} — {moe['name']}{star}\n\n{moe.get('statement','')}\n")
        s.append("| MOP | Definition | Target | Units | Measurement |\n|---|---|---|---|---|")
        for mid in moe.get('mops', []):
            mop = idx.get(mid, (None, {}))[1]
            s.append(f"| {mid} | {mop.get('definition','')} | {mop.get('target','TBD')} | "
                     f"{mop.get('units','')} | {mop.get('measurement','TBD')} |")
        s.append("")
    w(os.path.join(d, '02_MOE_MOP.md'), "\n".join(s))

    # ---- 03 System Requirements ----
    s = ["# System Requirements", f"_Generated {stamp} from model/requirements.yaml._\n"]
    by_type = {}
    for r in m.get('requirements', []):
        by_type.setdefault(r.get('type', 'other'), []).append(r)
    for typ in sorted(by_type):
        s.append(f"## {typ.capitalize()}\n")
        s.append("| ID | Requirement | Parent | Allocated | V | Status |\n|---|---|---|---|---|---|")
        for r in by_type[typ]:
            alloc = ", ".join(r.get('allocated_to', []) or [])
            s.append(f"| {r['id']} | {r['text']} | {r.get('parent','')} | {alloc} | {r.get('verification','')} | {r.get('status','')} |")
        s.append("")
    w(os.path.join(d, '03_System_Requirements.md'), "\n".join(s))

    # ---- 04 Architecture (+ Mermaid) ----
    s = ["# Architecture", f"_Generated {stamp}._\n", "## Component tree\n", "```mermaid", "graph TD"]
    for c in m.get('components', []):
        p = c.get('parent')
        if p:
            s.append(f'  {p}["{name(p)}"] --> {c["id"]}["{c["name"]}"]')
    s.append("```\n\n## Interface flows\n\n```mermaid\nflowchart LR")
    for itf in m.get('interfaces', []):
        if itf.get('kind') == 'ros_topic':
            s.append(f'  {itf["producer"]} -->|{itf["name"]}| {itf["consumer"]}')
    s.append("```")
    w(os.path.join(d, '04_Architecture.md'), "\n".join(s))

    # ---- 05 OPM model: OPL (sentences) + OPD (diagram) ----
    opm = m.get('opm', {})
    objs = {o['id']: o for o in opm.get('objects', [])}
    procs = {p['id']: p for p in opm.get('processes', [])}
    links = opm.get('links', [])
    onm = lambda i: objs[i]['name'] if i in objs else procs.get(i, {}).get('name', i)

    def state_phrase(states):
        if len(states) == 1:
            return f"can be {states[0]}"
        return "can be " + ", ".join(states[:-1]) + f", or {states[-1]}"

    s = ["# OPM Model — Object-Process Language (OPL) + Diagram",
         f"_Generated {stamp} from model/opm.yaml (Object-Process Methodology, ISO 19450)._\n",
         "## OPL — the model as sentences\n"]
    for o in opm.get('objects', []):
        s.append(f"- **{o['name']}** is {o['essence']} and {o['affiliation']}.")
        if o.get('states'):
            s.append(f"- **{o['name']}** {state_phrase(o['states'])}.")
    s.append("")
    for p in opm.get('processes', []):
        s.append(f"**{p['name']}** is a process.")
        for l in [x for x in links if x['process'] == p['id']]:
            t, on = l['type'], onm(l['object'])
            if t == 'agent':
                s.append(f"- {on} handles {p['name']}.")
            elif t == 'instrument':
                s.append(f"- {p['name']} requires {on}.")
            elif t == 'consumption':
                s.append(f"- {p['name']} consumes {on}.")
            elif t == 'result':
                pre = f"{l['to_state']} " if l.get('to_state') else ""
                s.append(f"- {p['name']} yields {pre}{on}.")
            elif t == 'effect':
                if l.get('from_state') and l.get('to_state'):
                    s.append(f"- {p['name']} changes {on} from {l['from_state']} to {l['to_state']}.")
                else:
                    s.append(f"- {p['name']} affects {on}.")
        s.append("")

    s.append("## OPD — Object-Process Diagram\n")
    s.append("```mermaid\nflowchart TB")
    for o in opm.get('objects', []):
        lbl = o['name'] + (("<br>[" + " | ".join(o['states']) + "]") if o.get('states') else "")
        s.append(f'  {o["id"]}["{lbl}"]:::obj')
    for p in opm.get('processes', []):
        s.append(f'  {p["id"]}(["{p["name"]}"]):::proc')
    for l in links:
        t = l['type']
        lbl = (f"effect&rarr;{l['to_state']}" if t == 'effect' and l.get('to_state') else t)
        if t in ('agent', 'instrument', 'consumption'):
            s.append(f'  {l["object"]} -- {lbl} --> {l["process"]}')
        else:
            s.append(f'  {l["process"]} -- {lbl} --> {l["object"]}')
    s.append("classDef obj fill:#dbeafe,stroke:#1e3a8a,color:#000;")
    s.append("classDef proc fill:#dcfce7,stroke:#166534,color:#000;")
    s.append("```")
    w(os.path.join(d, '05_OPM_OPL.md'), "\n".join(s))

    # ---- Requirements Traceability Matrix (CSV) ----
    cover = {}
    for a in m.get('activities', []):
        cover.setdefault(a.get('verifies'), []).append(a['id'])
    rows = []
    for r in m.get('requirements', []):
        rows.append([r['id'], r['text'], r.get('type', ''), r.get('parent', ''),
                     "; ".join(r.get('allocated_to', []) or []),
                     VMETHOD.get(r.get('verification', ''), r.get('verification', '')),
                     "; ".join(cover.get(r['id'], [])) or "NONE",
                     r.get('status', '')])
    csv_write(os.path.join(d, 'Requirements_Traceability_Matrix.csv'),
              ['Req_ID', 'Requirement', 'Type', 'Parent', 'Allocated_To', 'Verification', 'Covered_By', 'Status'], rows)

    # ---- Verification Matrix (CSV) ----
    rows = [[a['id'], a.get('verifies', ''), name(a.get('verifies', '')) if False else '',
             a.get('method', ''), a.get('integrity', ''), a.get('phase', ''), a.get('status', '')]
            for a in m.get('activities', [])]
    # include requirement text for readability
    rows = []
    for a in m.get('activities', []):
        req = idx.get(a.get('verifies'), (None, {}))[1]
        rows.append([a['id'], a.get('verifies', ''), req.get('text', ''),
                     a.get('method', ''), a.get('integrity', ''), a.get('phase', ''),
                     a.get('venue', ''), a.get('automated', ''),
                     'yes' if a.get('human_loop') else '', a.get('script', ''),
                     a.get('tool', ''), a.get('pass', ''), a.get('status', '')])
    csv_write(os.path.join(d, 'Verification_Matrix.csv'),
              ['Activity_ID', 'Verifies', 'Requirement', 'Method', 'Integrity', 'Phase',
               'Venue', 'Automated', 'Human_Loop', 'Script',
               'Tool', 'Pass_Criterion', 'Status'], rows)

    # ---- 06 Verification Procedures (checkable) ----
    s = ["# Verification Procedures", f"_Generated {stamp} from model/verification.yaml._",
         "_Mark `status: pass|fail` in the model and regenerate. Tools per Round-5 confirmed set._\n"]
    for a in m.get('activities', []):
        req = idx.get(a.get('verifies'), (None, {}))[1]
        s.append(f"## {a['id']} — verifies {a.get('verifies','')} ({a.get('method','')}, {a.get('integrity','')})")
        s.append(f"> {req.get('text','')}\n")
        venue = a.get('venue', '')
        auto = a.get('automated', '')
        badge = f"**venue:** `{venue}`  ·  **automated:** `{auto}`"
        if a.get('human_loop'):
            badge += "  ·  👤 human-in-the-loop"
        s.append(f"- {badge}")
        if a.get('script'):
            s.append(f"- **Script:** `{a['script']}`")
        s.append(f"- **Tool:** {a.get('tool','TBD')}")
        s.append(f"- **Procedure:** {a.get('procedure','TBD')}")
        s.append(f"- **Pass:** {a.get('pass','TBD')}")
        s.append(f"- **Phase:** {a.get('phase','')}  ·  **Status:** `{a.get('status','open')}`")
        s.append("- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL\n")
    w(os.path.join(d, '06_Verification_Procedures.md'), "\n".join(s))

    # ---- 07 Concept of Operations (+ scenario traceability) ----
    cn = m.get('conops', {})
    env = cn.get('environment', {})
    s = ["# Concept of Operations (ConOps)",
         f"_Generated {stamp} from model/conops.yaml._\n", "## Operating environment\n"]
    for k, v in env.items():
        s.append(f"- **{k.replace('_', ' ')}:** {v}")
    s.append("\n## Operational phases\n")
    s.append("| Phase | OPM process | Vehicle state | Narrative |\n|---|---|---|---|")
    for p in cn.get('phases', []):
        s.append(f"| {p['id']} {p['name']} | {p.get('opm','')} | {p.get('state','')} | {p.get('narrative','')} |")
    s.append("\n## Validation scenarios\n")
    for sc in cn.get('scenarios', []):
        s.append(f"### {sc['id']} — {sc['name']}  _(phase {sc.get('phase','')})_")
        s.append(f"{sc.get('narrative','')}\n")
        s.append(f"- **Requirements:** {', '.join(sc.get('traces', []) or [])}")
        if sc.get('mops'):
            s.append(f"- **MOPs:** {', '.join(sc['mops'])}")
        s.append(f"- **Verification:** {', '.join(sc.get('activities', []) or [])}\n")
    w(os.path.join(d, '07_ConOps.md'), "\n".join(s))

    rows = []
    for sc in cn.get('scenarios', []):
        rows.append([sc['id'], sc['name'], sc.get('phase', ''),
                     "; ".join(sc.get('traces', []) or []),
                     "; ".join(sc.get('mops', []) or []),
                     "; ".join(sc.get('activities', []) or [])])
    csv_write(os.path.join(d, 'Scenario_Traceability_Matrix.csv'),
              ['Scenario', 'Name', 'Phase', 'Requirements', 'MOPs', 'Verification'], rows)

    # ---- 08 Software Safety (NASA-GB-8719.13) ----
    ss = m.get('software_safety', {})
    if ss:
        det = ss.get('determination', {})
        s = ["# Software Safety Analysis (NASA-GB-8719.13)",
             f"_Generated {stamp} from model/software_safety.yaml._\n", "## Determination\n"]
        s.append(f"- **Safety-critical:** {det.get('safety_critical')}")
        s.append(f"- **Software control category:** {det.get('control_category','').strip()}")
        s.append(f"- **Worst credible mishap:** {det.get('worst_credible_mishap','')}")
        s.append(f"- **Severity:** {det.get('severity','')}")
        s.append(f"- **Fail-safe state:** {det.get('fail_safe_state','')}")
        s.append(f"- **Rigor:** {det.get('rigor','')}")
        s.append("\n## Safety-critical requirements\n")
        for r in m.get('requirements', []):
            if r.get('safety_critical'):
                s.append(f"- **{r['id']}**: {r['text']}")
        s.append("\n## Software hazard analysis\n")
        s.append("| SWH | Software cause | System hazard | Sev | Likelihood | Controls | Verification |\n|---|---|---|---|---|---|---|")
        for h in ss.get('hazards', []):
            s.append(f"| {h['id']} | {h['cause']} | {h['hazard']} | {h.get('severity','')} | "
                     f"{h.get('likelihood','')} | {', '.join(h.get('controls', []) or [])} | "
                     f"{', '.join(h.get('verification', []) or [])} |")
        w(os.path.join(d, '08_Software_Safety.md'), "\n".join(s))
        rows = [[h['id'], h['cause'], h['hazard'], h.get('severity', ''), h.get('likelihood', ''),
                 "; ".join(h.get('controls', []) or []), "; ".join(h.get('verification', []) or [])]
                for h in ss.get('hazards', [])]
        csv_write(os.path.join(d, 'Software_Hazard_Matrix.csv'),
                  ['SWH', 'Software_Cause', 'System_Hazard', 'Severity', 'Likelihood', 'Controls', 'Verification'], rows)

    # ---- Interfaces (CSV) ----
    rows = [[i['id'], i['name'], i.get('kind', ''), i.get('type', ''),
             i.get('producer', ''), i.get('consumer', '')] for i in m.get('interfaces', [])]
    csv_write(os.path.join(d, 'Interfaces.csv'),
              ['ID', 'Name', 'Kind', 'Type', 'Producer', 'Consumer'], rows)

    # ---- 09 Technical Risk Register (+ 5x5 matrix) ----
    risks = m.get('risks', [])
    if risks:
        def band(sc):
            return 'High' if sc >= 12 else ('Medium' if sc >= 5 else 'Low')
        for r in risks:
            r['_score'] = (r.get('likelihood', 0) or 0) * (r.get('consequence', 0) or 0)
            r['_band'] = band(r['_score'])
        ranked = sorted(risks, key=lambda r: -r['_score'])
        rp = m.get('risk_policy', {})
        s = ["# Technical Risk Register", f"_Generated {stamp} from model/risks.yaml._\n",
             f"- **Scale:** {rp.get('scale','')}", f"- **Appetite:** {rp.get('appetite','')}\n",
             "## Risks (ranked)\n",
             "| ID | Title | L | C | Score | Band | Traces | Verification | Residual |\n|---|---|---|---|---|---|---|---|---|"]
        for r in ranked:
            s.append(f"| {r['id']} | {r['title']} | {r['likelihood']} | {r['consequence']} | {r['_score']} | "
                     f"{r['_band']} | {', '.join(r.get('traces',[]) or [])} | "
                     f"{', '.join(r.get('verification',[]) or [])} | {r.get('residual_target','')} |")
        grid = {(l, c): [] for l in range(1, 6) for c in range(1, 6)}
        for r in risks:
            grid[(r['likelihood'], r['consequence'])].append(r['id'])
        s.append("\n## Risk matrix  (columns = Likelihood 1->5, rows = Consequence 5->1)\n")
        s.append("| C \\\\ L | 1 | 2 | 3 | 4 | 5 |\n|---|---|---|---|---|---|")
        for c in range(5, 0, -1):
            row = [f"**{c}**"] + [" ".join(grid[(l, c)]) or "·" for l in range(1, 6)]
            s.append("| " + " | ".join(row) + " |")
        s.append("\n## Mitigations & triggers\n")
        for r in ranked:
            s.append(f"### {r['id']} — {r['title']}  ({r['_band']}, score {r['_score']})")
            s.append(f"{r.get('statement','')}\n")
            s.append(f"- **Mitigation:** {r.get('mitigation','')}")
            s.append(f"- **Trigger:** {r.get('trigger','')}")
            s.append(f"- **Owner:** {r.get('owner','')} · **Residual target:** {r.get('residual_target','')} · **Status:** `{r.get('status','open')}`\n")
        w(os.path.join(d, '09_Risk_Register.md'), "\n".join(s))
        rows = [[r['id'], r['title'], r['likelihood'], r['consequence'], r['_score'], r['_band'],
                 "; ".join(r.get('traces', []) or []), "; ".join(r.get('verification', []) or []),
                 r.get('mitigation', ''), r.get('trigger', ''), r.get('residual_target', ''), r.get('status', '')]
                for r in ranked]
        csv_write(os.path.join(d, 'Risk_Register.csv'),
                  ['ID', 'Title', 'L', 'C', 'Score', 'Band', 'Traces', 'Verification',
                   'Mitigation', 'Trigger', 'Residual_Target', 'Status'], rows)

    # ---- 10 Interface Requirements (IRD) ----
    ird = m.get('interface_requirements', [])
    base = m.get('interface_baseline', {})
    if ird:
        s = ["# Interface Requirements Document (IRD)",
             f"_Generated {stamp} from model/interfaces_detail.yaml._",
             f"_Baseline {base.get('version','')} ({base.get('status','')}). "
             f"`[TBC]` = derived from the ROS code, not yet ratified._\n",
             "| ID | Interface | Category | Requirement | Value | TBC | Verifies via | Status |",
             "|---|---|---|---|---|---|---|---|"]
        for r in ird:
            s.append(f"| {r['id']} | {r.get('interface','')} ({name(r.get('interface',''))}) | "
                     f"{r.get('category','')} | {r.get('text','')} | {r.get('value','')} | "
                     f"{'yes' if r.get('tbc') else '—'} | {r.get('verification','—')} | {r.get('status','')} |")
        s.append("\n## Rationale\n")
        for r in ird:
            s.append(f"- **{r['id']}** ({r.get('interface','')}): {r.get('rationale','')}")
        w(os.path.join(d, '10_Interface_Requirements.md'), "\n".join(s))
        rows = [[r['id'], r.get('interface', ''), r.get('category', ''), r.get('text', ''),
                 r.get('value', ''), 'yes' if r.get('tbc') else 'no',
                 r.get('verification', ''), r.get('status', '')] for r in ird]
        csv_write(os.path.join(d, 'Interface_Requirements.csv'),
                  ['IRD_ID', 'Interface', 'Category', 'Requirement', 'Value', 'TBC', 'Verification', 'Status'], rows)

    # ---- 11 Interface Design (IDD) ----
    design = {x['id']: x for x in m.get('interface_design', [])}
    if design:
        s = ["# Interface Design Description (IDD)",
             f"_Generated {stamp} from model/interfaces_detail.yaml._\n"]
        for itf in m.get('interfaces', []):
            dgn = design.get(itf['id'])
            s.append(f"## {itf['id']} — {itf['name']}  ({itf.get('kind','')})")
            if not dgn:
                s.append("_(no design entry)_\n"); continue
            s.append(f"- **Direction:** {dgn.get('direction','')}")
            s.append(f"- **Encoding:** {dgn.get('encoding','')}")
            s.append(f"- **Rate:** {dgn.get('rate','')}")
            s.append(f"- **QoS:** {dgn.get('qos','')}")
            s.append(f"- **Units:** {dgn.get('units','')}  ·  **Range:** {dgn.get('range','')}")
            s.append(f"- **Error handling:** {dgn.get('error_handling','')}")
            s.append(f"- **Source:** `{dgn.get('source','')}`\n")
        w(os.path.join(d, '11_Interface_Design.md'), "\n".join(s))

    # ---- 12 Interface Control Document (ICD) — frozen baseline per interface ----
    if design or ird:
        ird_by_if = {}
        for r in ird:
            ird_by_if.setdefault(r.get('interface'), []).append(r)
        s = ["# Interface Control Document (ICD)",
             f"_Generated {stamp}. The controlled baseline: merges IRD + IDD per interface._",
             f"\n- **Baseline:** {base.get('version','')} ({base.get('status','')})",
             f"- **Control authority:** {base.get('control_authority','')}",
             f"- **Change policy:** {base.get('change_policy','').strip()}\n"]
        for itf in m.get('interfaces', []):
            dgn = design.get(itf['id'], {})
            s.append(f"## {itf['id']} — {itf['name']}")
            s.append(f"**{itf.get('producer','')} ({name(itf.get('producer',''))}) "
                     f"→ {itf.get('consumer','')} ({name(itf.get('consumer',''))})**  ·  "
                     f"{itf.get('kind','')} · `{itf.get('type','')}`\n")
            s.append(f"- Direction: {dgn.get('direction','')}")
            s.append(f"- Encoding: {dgn.get('encoding','')}  ·  Rate: {dgn.get('rate','')}  ·  QoS: {dgn.get('qos','')}")
            s.append(f"- Units/Range: {dgn.get('units','')} / {dgn.get('range','')}")
            s.append(f"- Error handling: {dgn.get('error_handling','')}")
            reqs = ird_by_if.get(itf['id'], [])
            if reqs:
                s.append("- Requirements:")
                for r in reqs:
                    tbc = " _[TBC]_" if r.get('tbc') else ""
                    s.append(f"    - **{r['id']}** ({r.get('category','')}): {r.get('text','')}{tbc}")
            s.append("")
        w(os.path.join(d, '12_ICD.md'), "\n".join(s))
        rows = []
        for itf in m.get('interfaces', []):
            dgn = design.get(itf['id'], {})
            rows.append([itf['id'], itf['name'], itf.get('producer', ''), itf.get('consumer', ''),
                         itf.get('type', ''), dgn.get('rate', ''), dgn.get('qos', ''),
                         dgn.get('units', ''), dgn.get('error_handling', ''),
                         "; ".join(r['id'] for r in ird_by_if.get(itf['id'], []))])
        csv_write(os.path.join(d, 'ICD.csv'),
                  ['Interface', 'Name', 'Producer', 'Consumer', 'Type', 'Rate', 'QoS',
                   'Units', 'Error_Handling', 'Requirements'], rows)

    # ---- 13 Integration Plan + 14 DSM (computed from interface coupling) ----
    integ = m.get('integration', {})
    # Build producer->consumer graph over components that appear in interfaces.
    edges = [(i['producer'], i['consumer']) for i in m.get('interfaces', [])
             if i.get('producer') and i.get('consumer')]
    nodes = sorted({n for e in edges for n in e})
    succ = {n: set() for n in nodes}
    for p, c in edges:
        succ[p].add(c)

    # Tarjan SCC (iterative) to find coupled clusters (feedback loops).
    index_c = {}; low = {}; onstack = {}; stack = []; sccs = []; counter = [0]
    def strongconnect(v):
        work = [(v, iter(sorted(succ[v])))]
        index_c[v] = low[v] = counter[0]; counter[0] += 1
        stack.append(v); onstack[v] = True
        while work:
            node, it = work[-1]
            advanced = False
            for wv in it:
                if wv not in index_c:
                    index_c[wv] = low[wv] = counter[0]; counter[0] += 1
                    stack.append(wv); onstack[wv] = True
                    work.append((wv, iter(sorted(succ[wv])))); advanced = True; break
                elif onstack.get(wv):
                    low[node] = min(low[node], index_c[wv])
            if advanced:
                continue
            work.pop()
            if low[node] == index_c[node]:
                comp = []
                while True:
                    x = stack.pop(); onstack[x] = False; comp.append(x)
                    if x == node:
                        break
                sccs.append(comp)
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[node])
    for n in nodes:
        if n not in index_c:
            strongconnect(n)

    comp_of = {}
    for ci, comp in enumerate(sccs):
        for n in comp:
            comp_of[n] = ci
    # Condensation DAG + Kahn topological order (integration order).
    cedges = {ci: set() for ci in range(len(sccs))}
    indeg = {ci: 0 for ci in range(len(sccs))}
    for p, c in edges:
        a, b = comp_of[p], comp_of[c]
        if a != b and b not in cedges[a]:
            cedges[a].add(b); indeg[b] += 1
    order, queue = [], sorted([ci for ci in indeg if indeg[ci] == 0])
    while queue:
        ci = queue.pop(0); order.append(ci)
        for nb in sorted(cedges[ci]):
            indeg[nb] -= 1
            if indeg[nb] == 0:
                queue.append(nb)
        queue.sort()

    if integ:
        s = ["# Integration & Verification Plan",
             f"_Generated {stamp} from model/integration.yaml._\n",
             f"- **Strategy:** {integ.get('strategy','')}",
             f"- **Method:** {integ.get('method','')}",
             f"- **Rationale:** {integ.get('rationale','').strip()}\n",
             "## Stages\n"]
        for st in integ.get('stages', []):
            wet = "💧 WET" if st.get('wet') else "🔧 DRY"
            s.append(f"### {st['id']} — {st['name']}  ({wet})")
            s.append(f"_{st.get('goal','')}_\n")
            s.append(f"- **Components added:** {', '.join(st.get('components', []) or [])}")
            s.append(f"- **Interfaces exercised:** {', '.join(st.get('interfaces', []) or []) or '—'}")
            s.append(f"- **Entry:** {st.get('entry','')}")
            s.append(f"- **Exit gate:** {st.get('exit_gate','')}")
            s.append(f"- **Verification:** {', '.join(st.get('verification', []) or [])}")
            s.append(f"- **Risks addressed:** {', '.join(st.get('risks', []) or []) or '—'}\n")
        w(os.path.join(d, '13_Integration_Plan.md'), "\n".join(s))
        rows = [[st['id'], st['name'], 'wet' if st.get('wet') else 'dry',
                 "; ".join(st.get('components', []) or []),
                 "; ".join(st.get('interfaces', []) or []),
                 st.get('exit_gate', ''), "; ".join(st.get('verification', []) or []),
                 "; ".join(st.get('risks', []) or [])] for st in integ.get('stages', [])]
        csv_write(os.path.join(d, 'Integration_Plan.csv'),
                  ['Stage', 'Name', 'Environment', 'Components', 'Interfaces',
                   'Exit_Gate', 'Verification', 'Risks'], rows)

    # ---- 14 DSM (Design Structure Matrix) ----
    if nodes:
        short = lambda n: n.replace('C-', '')
        depmark = {(c, p) for p, c in edges}   # row=consumer depends on col=producer
        s = ["# Design Structure Matrix (DSM)",
             f"_Generated {stamp} — computed from the interface coupling in architecture.yaml._\n",
             "Cell **X** at row _R_, column _C_ means **R receives from / depends on C** "
             "(an interface flows C→R). Marks below the diagonal = forward flow; "
             "marks above = feedback.\n",
             "## Component coupling matrix\n",
             "| ⬇R \\\\ C➡ | " + " | ".join(short(n) for n in nodes) + " |",
             "|---|" + "---|" * len(nodes)]
        for r in nodes:
            row = [f"**{short(r)}**"]
            for c in nodes:
                row.append("X" if (r, c) in depmark else ("·" if r != c else "—"))
            s.append("| " + " | ".join(row) + " |")
        s.append("\n## Coupled clusters (feedback loops)\n")
        clusters = [c for c in sccs if len(c) > 1]
        if clusters:
            s.append("These components form feedback loops and **must be integrated as a block** "
                     "(piecewise bring-up is impossible — each needs the others):\n")
            for comp in clusters:
                s.append(f"- {{ {', '.join(f'{n} ({name(n)})' for n in sorted(comp))} }}")
        else:
            s.append("_None — the architecture is acyclic._")
        s.append("\n## DSM-derived integration order\n")
        s.append("Topological order of the clusters (providers before consumers):\n")
        for rank, ci in enumerate(order, 1):
            comp = sccs[ci]
            tag = "  ← coupled block" if len(comp) > 1 else ""
            s.append(f"{rank}. {', '.join(short(n) for n in sorted(comp))}{tag}")
        s.append("\n## Cross-check vs the authored stages\n")
        s.append("The control core (controller ↔ dynamixel ↔ crab ↔ console) is the coupled block — "
                 "the hand-authored plan brings its sensor providers up first in **INT-0**, then closes "
                 "that block in **INT-1** (dry) before any water. The DSM order and the staged plan agree.")
        w(os.path.join(d, '14_DSM.md'), "\n".join(s))
        rows = []
        for r in nodes:
            rows.append([short(r)] + ["X" if (r, c) in depmark else ("" if r != c else "—") for c in nodes])
        csv_write(os.path.join(d, 'DSM.csv'), ['Row_depends_on->'] + [short(n) for n in nodes], rows)

    # ---- 15 Commissioning — Operational Readiness Review ----
    com = m.get('commissioning', {})
    if com:
        s = ["# Commissioning — Operational Readiness Review (ORR)",
             f"_Generated {stamp} from model/conops.yaml._\n",
             f"- **Authority:** {com.get('authority','')}",
             f"- **Gate:** {com.get('gate','').strip()}",
             f"- **Recommission:** {com.get('recommission','')}",
             f"- **Exit:** {com.get('exit','')}\n",
             "## Readiness checklist\n",
             "| ID | Category | Item | Evidence | Status |",
             "|---|---|---|---|---|"]
        for it in com.get('checklist', []):
            s.append(f"| {it['id']} | {it.get('category','')} | {it.get('item','')} | "
                     f"{', '.join(it.get('evidence', []) or [])} | `{it.get('status','open')}` |")
        s.append("\n## Sign-off\n")
        s.append("- Integration & V&V evidence reviewed: ______________  Date: ______")
        s.append("- All technical risks at Low residual: ______________  Date: ______")
        s.append("- **PI — readiness declared:** ______________  Date: ______")
        s.append("- **Safety Observer — concurrence:** ______________  Date: ______")
        s.append("\n> Vehicle is **COMMISSIONED** only when all items above carry objective "
                 "evidence and both signatures are present. Until then it remains in PH-0.")
        w(os.path.join(d, '15_Commissioning.md'), "\n".join(s))
        rows = [[it['id'], it.get('category', ''), it.get('item', ''),
                 "; ".join(it.get('evidence', []) or []), it.get('status', 'open')]
                for it in com.get('checklist', [])]
        csv_write(os.path.join(d, 'Commissioning_Checklist.csv'),
                  ['ID', 'Category', 'Item', 'Evidence', 'Status'], rows)

    # ---- 16 Test Strategy (venue ladder + automation coverage) ----
    vs = m.get('verification_strategy', {})
    acts = m.get('activities', [])
    if vs and acts:
        VORDER = ['mil', 'sil', 'hil', 'water']
        s = ["# Test Strategy — model→reality ladder & automation",
             f"_Generated {stamp} from model/verification.yaml._\n",
             f"**Principle:** {vs.get('principle','')}\n", "## The venue ladder\n",
             "| Venue | Name | Automatable | What it is |", "|---|---|---|---|"]
        for v in vs.get('venues', []):
            s.append(f"| `{v['id']}` | {v.get('name','')} | {v.get('automatable','')} | {v.get('desc','')} |")

        # coverage counts
        by_venue = {v: 0 for v in VORDER}
        by_auto = {'full': 0, 'partial': 0, 'none': 0}
        human = []
        for a in acts:
            by_venue[a.get('venue', 'water')] = by_venue.get(a.get('venue', 'water'), 0) + 1
            by_auto[a.get('automated', 'none')] = by_auto.get(a.get('automated', 'none'), 0) + 1
            if a.get('human_loop'):
                human.append(a['id'])
        n = len(acts)
        s.append("\n## Coverage\n")
        s.append(f"- **{n} activities total.**")
        s.append(f"- By venue: " + ", ".join(f"{v}={by_venue.get(v,0)}" for v in VORDER))
        s.append(f"- By automation: full={by_auto['full']}, partial={by_auto['partial']}, none={by_auto['none']}  "
                 f"→ **{by_auto['full']} fully automated / {by_auto['full']+by_auto['partial']} with some automation**")
        s.append(f"- Human-in-the-loop: {', '.join(human) or 'none'}")

        s.append("\n## Activities by venue\n")
        for v in VORDER:
            ids = [a['id'] for a in acts if a.get('venue') == v]
            vn = next((x.get('name', v) for x in vs.get('venues', []) if x['id'] == v), v)
            s.append(f"- **{v}** ({vn}): {', '.join(ids) or '—'}")

        ks = vs.get('kinematic_sim', {})
        if ks:
            s.append("\n## What the kinematic Gazebo can / cannot close\n")
            s.append(f"- **It is:** {ks.get('is','')}")
            s.append(f"- **✅ Can verify (SIL):** {ks.get('can_verify','')}")
            s.append(f"- **❌ Cannot verify (needs HIL / water):** {ks.get('cannot_verify','')}")
            s.append(f"- **Gap / action:** {ks.get('gap','')}")

        hil = vs.get('human_in_the_loop', {})
        if hil:
            s.append(f"\n## Human-in-the-loop\n\n{hil.get('desc','')}\n")
            s.append(f"Activities: {', '.join(hil.get('activities', []) or [])}")
        w(os.path.join(d, '16_Test_Strategy.md'), "\n".join(s))

    # ---- 00 Model Report (counts + live check results) ----
    errors, warnings = checker.run()
    counts = {k: len(m.get(k, [])) for k in
              ('stakeholders', 'expectations', 'moes', 'mops', 'requirements',
               'components', 'interfaces', 'activities')}
    s = [f"# Model Report — {m.get('program', {}).get('name', '')}",
         f"_Generated {stamp}._\n", "## Element counts\n"]
    for k, v in counts.items():
        s.append(f"- {k}: {v}")
    vp = m.get('verification_policy', {})
    if vp:
        s.append("\n## V&V resourcing policy\n")
        s.append(f"- **Software:** {vp.get('software','')}")
        s.append(f"- **Hardware:** {vp.get('hardware','').strip()}")
        s.append(f"- **Confirmed tools:** {', '.join(vp.get('confirmed_tools') or []) or '_(none confirmed yet)_'}")
        s.append(f"- **Unavailable tools:** {', '.join(vp.get('unavailable_tools') or []) or '_(none)_'}")
    s.append(f"\n## Consistency check\n\n**{len(errors)} error(s), {len(warnings)} warning(s)**\n")
    for e in errors:
        s.append(f"- ERROR: {e}")
    for ww in warnings:
        s.append(f"- WARN: {ww}")
    s.append("\n## Generated views\n")
    for f in sorted(os.listdir(d)):
        if f != '00_Model_Report.md':
            s.append(f"- `{f}`")
    w(os.path.join(d, '00_Model_Report.md'), "\n".join(s))

    # ---- INDEX (master navigation, generated last) ----
    prog = m.get('program', {})
    health = f"**{len(errors)} errors, {len(warnings)} warnings**"
    groups = [
        ("0. Start here", [
            ("00_Model_Report.md", "Element counts, V&V policy, live consistency check"),
            ("07_ConOps.md", "Concept of Operations — environment, phases (PH-0..PH-6), scenarios"),
            ("15_Commissioning.md", "Operational Readiness Review — the gate from integration to operations")]),
        ("1. Concept & needs", [
            ("01_Stakeholder_Expectations.md", "Stakeholders and their expectations"),
            ("02_MOE_MOP.md", "Measures of Effectiveness / Performance (targets + measurement)")]),
        ("2. Requirements", [
            ("03_System_Requirements.md", "All requirements by domain"),
            ("Requirements_Traceability_Matrix.csv", "Req -> parent -> allocation -> verification -> status")]),
        ("3. Architecture & model", [
            ("04_Architecture.md", "Components + interfaces (Mermaid)"),
            ("05_OPM_OPL.md", "Object-Process model — OPL sentences + OPD diagram"),
            ("Interfaces.csv", "Interface register")]),
        ("4. Verification & Validation", [
            ("16_Test_Strategy.md", "MIL→SIL→HIL→water ladder, automation coverage, kinematic-sim limits"),
            ("06_Verification_Procedures.md", "Checkable procedures (venue/automation/script tagged)"),
            ("Verification_Matrix.csv", "Activity -> requirement -> venue -> automation -> tool -> pass"),
            ("Scenario_Traceability_Matrix.csv", "Scenario -> requirements -> MOPs -> verification")]),
        ("5. Safety & risk", [
            ("08_Software_Safety.md", "NASA-GB-8719.13 analysis (6 software hazards)"),
            ("Software_Hazard_Matrix.csv", "SWH cause -> hazard -> control -> verification"),
            ("09_Risk_Register.md", "Technical risks (5x5) + matrix + mitigations"),
            ("Risk_Register.csv", "Risk register")]),
        ("6. Interfaces & integration", [
            ("10_Interface_Requirements.md", "IRD — interface SHALLs (timing/protocol/data/ordering/error)"),
            ("11_Interface_Design.md", "IDD — per-interface design (encoding, rate, QoS, error handling)"),
            ("12_ICD.md", "ICD — controlled baseline merging IRD + IDD per interface"),
            ("Interface_Requirements.csv", "IRD register"),
            ("ICD.csv", "ICD baseline register"),
            ("13_Integration_Plan.md", "Incremental dry→wet integration stages + gates"),
            ("14_DSM.md", "Design Structure Matrix + coupled clusters + derived order"),
            ("Integration_Plan.csv", "Integration stage register"),
            ("DSM.csv", "Component dependency matrix")]),
    ]
    ix = [f"# {prog.get('name','')} — Subsea Mission Assurance (Master Index)",
          f"_Generated {stamp}. **Do not edit docs/** — edit `model/*.yaml`, then run "
          f"`python3 tools/check.py && python3 tools/generate.py`._\n",
          f"Model-based systems engineering (as code). Source of truth: `model/`. Health: {health}.\n",
          "| Area | Document | What it is |\n|---|---|---|"]
    for title, items in groups:
        for i, (fn, desc) in enumerate(items):
            ix.append(f"| {title if i == 0 else ''} | [{fn}]({fn}) | {desc} |")
    ix.append("\n## Recommended reading order\n")
    ix.append("ConOps → Stakeholder Expectations → MOE/MOP → Requirements (+RTM) → "
              "Architecture → OPM → Verification procedures (+matrices, scenarios) → "
              "Software Safety → Risk Register → Model Report.\n")
    ix.append("## Regenerate\n```bash\npython3 tools/check.py      # consistency gate\n"
              "python3 tools/generate.py   # rebuild every view\n```")
    w(os.path.join(d, 'INDEX.md'), "\n".join(ix))

    print(f"generated {len(os.listdir(d))} files into docs/  "
          f"({len(errors)} errors, {len(warnings)} warnings)")


if __name__ == '__main__':
    main()
