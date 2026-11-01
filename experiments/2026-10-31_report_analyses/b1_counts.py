"""Exact counts behind the text of Section 6.3 (from b1_energy_prediction.npz)."""
import numpy as np

b = np.load('b1_energy_prediction.npz')
E_LAND = 3387.0
go_under = b['go_meas'] > b['go_pred']
print('E_go charged: transits', len(b['go_pred']), 'under', go_under.sum(),
      'by', np.round(b['go_meas'][go_under] - b['go_pred'][go_under]))
print('E_go charged above measured: median J',
      np.median(b['go_pred'] - b['go_meas']).round(),
      'median % of charge',
      np.median((b['go_pred'] - b['go_meas']) / b['go_pred'] * 100).round(1))
ret_c = b['ret_pred'] + b['ret_tau']
ru = b['ret_meas'] > ret_c
print('E_ret + tau: legs', len(ret_c), 'under', ru.sum(), 'by',
      np.round(b['ret_meas'][ru] - ret_c[ru]), 'lengths', b['ret_len'][ru])
land = b['land']
print('E_land: n', len(land), 'measured above 3387:', (land > E_LAND).sum(),
      'by', np.round(land[land > E_LAND] - E_LAND), 'median', np.median(land))
tot_pred = ret_c + E_LAND
tot_meas = b['ret_meas'] + land
tu = tot_meas > tot_pred
print('return + landing: under', tu.sum(), 'of', len(tot_pred), 'by',
      np.round(tot_meas[tu] - tot_pred[tu]))
