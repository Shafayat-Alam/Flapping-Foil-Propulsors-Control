#!/usr/bin/env python3
"""
check.py — model consistency checker (the heart of MBSE-as-code).

Validates cross-references and SE completeness across the model. ERRORS make
the model invalid (exit 1); WARNINGS are quality issues (exit 0). Run before
generating, and wire into CI so the model can't silently drift.

Usage:  python3 tools/check.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _model

VMETHODS = {'T', 'A', 'I', 'D', 'Test', 'Analysis', 'Inspection', 'Demonstration'}


def run():
    m = _model.load()
    idx = _model.index(m)
    errors, warnings = [], []

    def kind_of(i):
        return idx.get(i, (None, None))[0]

    # ---- duplicate IDs ----
    seen = {}
    for coll in ('moes', 'mops', 'stakeholders', 'expectations', 'requirements',
                 'components', 'interfaces', 'activities', 'external_actors'):
        for it in m.get(coll, []):
            i = it.get('id')
            if i in seen:
                errors.append(f"duplicate id '{i}' (in {coll} and {seen[i]})")
            seen[i] = coll
    for el in m.get('system_of_interest', {}).get('contains', []):
        if el['id'] in seen:
            errors.append(f"duplicate id '{el['id']}'")
        seen[el['id']] = 'elements'

    # ---- MOP -> MOE ----
    for mop in m.get('mops', []):
        if kind_of(mop.get('moe')) != 'moe':
            errors.append(f"MOP {mop['id']}: moe '{mop.get('moe')}' is not a known MOE")
    for moe in m.get('moes', []):
        if not moe.get('mops'):
            warnings.append(f"MOE {moe['id']} has no MOPs")

    # ---- expectations -> stakeholders ----
    for e in m.get('expectations', []):
        if kind_of(e.get('stakeholder')) != 'stakeholder':
            errors.append(f"Expectation {e['id']}: stakeholder '{e.get('stakeholder')}' unknown")

    # ---- components -> parent ----
    for c in m.get('components', []):
        p = c.get('parent')
        if p and kind_of(p) not in ('component', 'element'):
            errors.append(f"Component {c['id']}: parent '{p}' is not a component/element")

    # ---- interfaces -> producer/consumer ----
    for itf in m.get('interfaces', []):
        for role in ('producer', 'consumer'):
            if kind_of(itf.get(role)) != 'component':
                errors.append(f"Interface {itf['id']}: {role} '{itf.get(role)}' is not a component")

    # ---- requirements ----
    derived_parents = set()
    for r in m.get('requirements', []):
        p = r.get('parent')
        derived_parents.add(p)
        if kind_of(p) not in ('expectation', 'moe', 'mop', 'requirement'):
            errors.append(f"Req {r['id']}: parent '{p}' must trace to an expectation/MOE/MOP/req")
        if r.get('verification') not in VMETHODS:
            errors.append(f"Req {r['id']}: verification '{r.get('verification')}' not one of {sorted(VMETHODS)}")
        for a in r.get('allocated_to', []) or []:
            if kind_of(a) not in ('component', 'element'):
                errors.append(f"Req {r['id']}: allocated_to '{a}' is not a component/element")
        if not r.get('allocated_to'):
            warnings.append(f"Req {r['id']} is not allocated to any component")

    # ---- verification activities -> requirements (+ venue/automation) ----
    VENUES = {'mil', 'sil', 'hil', 'water'}
    AUTOLEVELS = {'full', 'partial', 'none'}
    verified = set()
    for a in m.get('activities', []):
        v = a.get('verifies')
        verified.add(v)
        if kind_of(v) != 'requirement':
            errors.append(f"Activity {a['id']}: verifies '{v}' is not a requirement")
        if 'venue' in a and a['venue'] not in VENUES:
            errors.append(f"Activity {a['id']}: venue '{a['venue']}' not one of {sorted(VENUES)}")
        if 'automated' in a and a['automated'] not in AUTOLEVELS:
            errors.append(f"Activity {a['id']}: automated '{a['automated']}' not one of {sorted(AUTOLEVELS)}")
        if a.get('automated') == 'full' and not a.get('script'):
            warnings.append(f"Activity {a['id']} is automated:full but names no script")

    # ---- coverage warnings ----
    for r in m.get('requirements', []):
        if r['id'] not in verified:
            warnings.append(f"Req {r['id']} has no verification activity (V&V gap)")
    for e in m.get('expectations', []):
        if e['id'] not in derived_parents:
            warnings.append(f"Expectation {e['id']} has no derived requirement (orphan need)")

    # ---- ConOps scenarios -> requirements / mops / activities ----
    for sc in m.get('conops', {}).get('scenarios', []):
        for r in sc.get('traces', []) or []:
            if kind_of(r) != 'requirement':
                errors.append(f"Scenario {sc['id']}: trace '{r}' is not a requirement")
        for mp in sc.get('mops', []) or []:
            if kind_of(mp) != 'mop':
                errors.append(f"Scenario {sc['id']}: mop '{mp}' is not a MOP")
        for ac in sc.get('activities', []) or []:
            if kind_of(ac) != 'activity':
                errors.append(f"Scenario {sc['id']}: activity '{ac}' is not a verification activity")

    # ---- Technical risks ----
    for r in m.get('risks', []):
        for fld in ('likelihood', 'consequence'):
            if not isinstance(r.get(fld), int) or not (1 <= r.get(fld, 0) <= 5):
                errors.append(f"Risk {r['id']}: {fld} must be an integer 1-5")
        for t in r.get('traces', []) or []:
            if kind_of(t) != 'requirement':
                errors.append(f"Risk {r['id']}: trace '{t}' is not a requirement")
        for v in r.get('verification', []) or []:
            if kind_of(v) != 'activity':
                errors.append(f"Risk {r['id']}: verification '{v}' is not an activity")

    # ---- V&V tool availability cross-check ----
    unavailable = m.get('verification_policy', {}).get('unavailable_tools', []) or []
    # match on the leading keyword of each unavailable entry (before any '(' or '/')
    keys = [u.split('(')[0].split('/')[0].strip().lower() for u in unavailable]
    for a in m.get('activities', []):
        tool = (a.get('tool') or '').lower()
        for k, full in zip(keys, unavailable):
            if k and k in tool:
                warnings.append(f"Activity {a['id']} tool references an UNAVAILABLE instrument "
                                f"('{full}') — substitute or mark BLOCKED")

    # ---- Software safety: SWH controls/verification + safety-critical coverage ----
    for h in m.get('software_safety', {}).get('hazards', []):
        if not h.get('controls'):
            errors.append(f"SWH {h['id']} has no control requirement")
        for c in h.get('controls', []) or []:
            if kind_of(c) != 'requirement':
                errors.append(f"SWH {h['id']}: control '{c}' is not a requirement")
        for v in h.get('verification', []) or []:
            if kind_of(v) != 'activity':
                errors.append(f"SWH {h['id']}: verification '{v}' is not an activity")
    for r in m.get('requirements', []):
        if r.get('safety_critical') and r['id'] not in verified:
            errors.append(f"Safety-critical {r['id']} has NO verification activity")

    # ---- OPM model consistency ----
    opm = m.get('opm', {})
    opm_obj = {o['id']: o for o in opm.get('objects', [])}
    opm_proc = {p['id'] for p in opm.get('processes', [])}
    LINK_TYPES = {'agent', 'instrument', 'consumption', 'result', 'effect'}
    referenced = set()
    for l in opm.get('links', []):
        if l.get('process') not in opm_proc:
            errors.append(f"OPM link: process '{l.get('process')}' is not a known process")
        if l.get('object') not in opm_obj:
            errors.append(f"OPM link: object '{l.get('object')}' is not a known object")
        if l.get('type') not in LINK_TYPES:
            errors.append(f"OPM link ({l.get('process')}->{l.get('object')}): type '{l.get('type')}' invalid")
        referenced.add(l.get('object'))
        ob = opm_obj.get(l.get('object'), {})
        for sk in ('from_state', 'to_state'):
            if l.get(sk) and l[sk] not in (ob.get('states') or []):
                errors.append(f"OPM link ({l.get('process')}->{l.get('object')}): "
                              f"{sk} '{l[sk]}' not a state of {l.get('object')}")
    for oid in opm_obj:
        if oid not in referenced:
            warnings.append(f"OPM object {oid} participates in no process link")

    # ---- Interface engineering: IDD design, IRD requirements ----
    iface_ids = {i['id'] for i in m.get('interfaces', [])}
    designed = set()
    for dgn in m.get('interface_design', []):
        i = dgn.get('id')
        if i in designed:
            errors.append(f"interface_design: duplicate entry for '{i}'")
        designed.add(i)
        if i not in iface_ids:
            errors.append(f"interface_design '{i}' is not a known interface")
    for itf in m.get('interfaces', []):
        if itf['id'] not in designed:
            warnings.append(f"Interface {itf['id']} has no IDD design entry")

    ird_seen = set()
    for r in m.get('interface_requirements', []):
        i = r.get('id')
        if i in ird_seen:
            errors.append(f"duplicate id '{i}' (interface_requirements)")
        ird_seen.add(i)
        if r.get('interface') not in iface_ids:
            errors.append(f"IRD {i}: interface '{r.get('interface')}' is not a known interface")
        if r.get('category') not in {'timing', 'protocol', 'data', 'ordering', 'error'}:
            errors.append(f"IRD {i}: category '{r.get('category')}' invalid")
        v = r.get('verification')
        if v and kind_of(v) != 'activity':
            errors.append(f"IRD {i}: verification '{v}' is not a known activity")
    for itf in m.get('interfaces', []):
        if not any(r.get('interface') == itf['id'] for r in m.get('interface_requirements', [])):
            warnings.append(f"Interface {itf['id']} has no IRD requirement")

    # ---- Integration plan: stage references must resolve ----
    risk_ids = {r['id'] for r in m.get('risks', [])}
    stage_ids = {st.get('id') for st in m.get('integration', {}).get('stages', [])}
    int_seen = set()
    for st in m.get('integration', {}).get('stages', []):
        i = st.get('id')
        if i in int_seen:
            errors.append(f"duplicate id '{i}' (integration stages)")
        int_seen.add(i)
        for c in st.get('components', []) or []:
            if kind_of(c) not in ('component', 'element'):
                errors.append(f"Stage {i}: component '{c}' is not a component/element")
        for itf in st.get('interfaces', []) or []:
            if itf not in iface_ids:
                errors.append(f"Stage {i}: interface '{itf}' is not a known interface")
        for a in st.get('verification', []) or []:
            if kind_of(a) != 'activity':
                errors.append(f"Stage {i}: verification '{a}' is not a known activity")
        for rk in st.get('risks', []) or []:
            if rk not in risk_ids:
                errors.append(f"Stage {i}: risk '{rk}' is not a known risk")

    # ---- Commissioning checklist: evidence must resolve (activity / stage / risk) ----
    activity_ids = {a['id'] for a in m.get('activities', [])}
    com_seen = set()
    for item in m.get('commissioning', {}).get('checklist', []):
        i = item.get('id')
        if i in com_seen:
            errors.append(f"duplicate id '{i}' (commissioning checklist)")
        com_seen.add(i)
        if item.get('category') not in {'readiness', 'safety', 'calibration', 'risk'}:
            errors.append(f"Commissioning {i}: category '{item.get('category')}' invalid")
        ev = item.get('evidence') or []
        if not ev:
            errors.append(f"Commissioning {i} has no evidence reference")
        for e in ev:
            if e not in activity_ids and e not in stage_ids and e not in risk_ids:
                errors.append(f"Commissioning {i}: evidence '{e}' is not a known activity/stage/risk")

    # ---- Verification strategy: human-in-the-loop list must resolve ----
    for hid in m.get('verification_strategy', {}).get('human_in_the_loop', {}).get('activities', []) or []:
        if hid not in activity_ids:
            errors.append(f"verification_strategy human-in-the-loop '{hid}' is not a known activity")

    return errors, warnings


def main():
    errors, warnings = run()
    for w in warnings:
        print(f"  WARN  {w}")
    for e in errors:
        print(f"  ERROR {e}")
    print(f"\nmodel check: {len(errors)} error(s), {len(warnings)} warning(s)")
    sys.exit(1 if errors else 0)


if __name__ == '__main__':
    main()
