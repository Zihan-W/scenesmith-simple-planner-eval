"""One secant probe derived from measured-scene geometric gap evidence."""
import json,subprocess,sys
from pathlib import Path
root=Path('runs/contact-parking-diagnostic-20260920')
a=json.loads((root/'local_seed500_v2/frozen_actions.json').read_text())[1]['geometric_parameters']
b=json.loads((root/'local_seed500_centered/repeat0/counterfactual_grasp_check.json').read_text())
pa=a['grasp_lateral_offset_m'];pb=b['parameters']['grasp_lateral_offset_m']
da=a['checks']['ik']['grasp_contacts']['open_finger_distances_m'];db=b['checks']['ik']['grasp_contacts']['open_finger_distances_m'];names=sorted(da)
ga=da[names[0]]-da[names[1]];gb=db[names[0]]-db[names[1]]
p=pb-gb*(pb-pa)/(gb-ga)
assert -.02<=p<=.02
(root/'balance_probe_protocol_v2.json').write_text(json.dumps({'mode':'DIAGNOSTIC_FIXED_GRASP','online_model_success_trial':False,'source_parameters':[pa,pb],'source_signed_gaps_m':[ga,gb],'secant_offset_m':p,'new_full_witness_budget':1,'new_dynamic_rollouts':1,'control_or_threshold_changes':False},indent=2))
raise SystemExit(subprocess.call([sys.executable,'scripts/diagnose_pick_contact.py','--seed','500','--repeat','1','--grasp-offset',str(p),'--output',str(root/'local_seed500_gap_secant_v2')]))
