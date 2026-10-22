# I4 per cell against I4 per touchdown point, offline, 22 Oct 2026

I4 (spec 3.4): every zone of the registry has been observed from at
least two distinct passes. The registry counts passes around one
touchdown point per zone, the cell farthest from refused ground: an
entry of the aircraft within r_in = 0.8 x 18 m of that point after it
was beyond 18 m. In the first W3 flight (`demos/2026-10-22_10-49_w3`)
the road was never confirmed: its touchdown fell 50 m west of the
take-off and was flown near once, so C1 and C2 relied on the yard
alone, and the return went 52 m to the yard when the road was 26 m
away. Dorian agreed on 22 Oct to measure a per-cell reading of I4.

## Variant

The same rule on every cell: a cell has been passed when the aircraft
entered within r_in of it after being beyond 18 m. The confirmable part
of a zone is its landable cells passed at least twice, kept where they
hold a whole footprint (the min-zone rule). Per cell the count can only
grow, which subsumes Dorian's decision of 19 Oct that a zone's pass
count never decreases.

## Method

`replay.py`, offline, from the logs alone: the track at 1 Hz from
cycles.jsonl (as the registry samples it) and the map snapshots (every
10 s). On each snapshot the manager's chain runs again (observation
states, terrain layers, landing detection on OBSERVED cells), and two
confirmed sets are compared: point, the non-provisional zones CONFIRMED
in the mission's log (their cells taken as the detected components
holding their touchdown); cell, the variant. Against the truth: cells
whose real slope exceeds 12 deg or real step 0.48 m (severe, as
`tools/validate_slz.py`), the S8 barrier. Open loop: the decisions of
the missions stay those taken with the point rule.

    python3 replay.py w3 ../../demos/2026-10-22_10-49_w3/missions/*
    python3 replay.py w2 .../2026-10-21_campaign_w2/runs/B3a*/missions/*
    python3 replay.py w1 .../2026-10-20_campaign_w1_fix/runs/B[23]*/missions/*

## Results

| | missions | first confirmed, median | area confirmed at the end, median | missions with a severe cell confirmed |
|---|---|---|---|---|
| W3, point | 1 | 55 s | 178 m2 (yard) | 0 |
| W3, cell | 1 | 44 s | 358 m2 (yard, road 143, Z 24, parking 13) | 0 |
| W2, point | 50 | 82 s | 142 m2 | 0 |
| W2, cell | 50 | 42 s | 142 m2 | 0 |
| W1, point | 60 | 33 s | **2465 m2** | 1 (B2_r03, 1 cell) |
| W1, cell | 60 | 32 s | **538 m2** | 0 |

- **W3**: per cell, a road section 18 m long, pad Z and part of the
  parking lot are confirmed, besides the yard: the parts of them flown
  near twice.
- **W2**: the first zone (the take-off pad) is confirmed earlier, 42
  against 82 s; no gain in area, the remote pads are flown near twice in almost no
  mission. In B3a100_r07 the point rule had confirmed pad_ne, whose
  cells were not all passed twice: per cell it stays a candidate.
- **W1**: the point rule over-confirms. Two passes near the touchdown
  of the large flat zone confirmed all of it, while 538 of its 2465 m2
  had been passed twice; the rest was seen in one pass, which I4 does
  not allow. The one severe cell the point rule confirmed (B2_r03) is
  not confirmed per cell (whether it was passed once or fell outside a
  whole footprint was not separated).

The last map of the W3 flight holds no severe candidate cell: the
short-lived landing fragments of the rock zone are safe ground in the
truth. In W1, 15 of the 60 last maps hold severe candidate cells, the
smoothed rough zone already known (spec section 10); none is confirmed
per cell.

## Conclusion

Per cell is the faithful reading of I4, and it moves both ways: more
zones where one touchdown point was badly placed (W3), less area where
one point stood for a large zone (W1). Next: in the registry, with the
return field seeding only the confirmed cells, measured in flight
before and after.
