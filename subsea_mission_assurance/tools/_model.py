"""
_model.py — shared model loader/indexer for the subsea_mission_assurance
model-based-as-code package. The model is the YAML files in ../model/;
check.py and generate.py both build on this.
"""
import os, glob, yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(ROOT, 'model')
DOCS_DIR = os.path.join(ROOT, 'docs')


def load():
    """Merge every model/*.yaml into one dict (lists concat, dicts merge)."""
    model = {}
    for path in sorted(glob.glob(os.path.join(MODEL_DIR, '*.yaml'))):
        data = yaml.safe_load(open(path)) or {}
        for key, val in data.items():
            if isinstance(val, list):
                model.setdefault(key, []).extend(val)
            elif isinstance(val, dict):
                model.setdefault(key, {}).update(val)
            else:
                model[key] = val
    return model


def index(model):
    """Return {id: ('kind', element)} across all id-bearing collections."""
    idx = {}
    collections = {
        'element': model.get('system_of_interest', {}).get('contains', []),
        'actor': model.get('external_actors', []),
        'moe': model.get('moes', []),
        'mop': model.get('mops', []),
        'stakeholder': model.get('stakeholders', []),
        'expectation': model.get('expectations', []),
        'requirement': model.get('requirements', []),
        'component': model.get('components', []),
        'interface': model.get('interfaces', []),
        'activity': model.get('activities', []),
    }
    for kind, items in collections.items():
        for it in items:
            if isinstance(it, dict) and 'id' in it:
                idx[it['id']] = (kind, it)
    return idx
