"""How many W2 C1 missions confirmed more than one discovered zone? (claim check)"""
import csv

rows = [r for r in csv.DictReader(open('b7_mission_metric.csv'))
        if r['world'] == 'W2' and r['config'].startswith('B3')]
more = [(r['config'], r['run'], r['n_zones']) for r in rows
        if r['n_zones'] != '1']
print(len(rows), 'W2 C1 missions; with a zone count other than 1:', more)
