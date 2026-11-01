"""
Report text, second half: Sections 5 to 7, the Gen-AI declaration
placeholder, references and the communication log placeholder.

Numbers: experiments/2026-10-23_campaign_w{1,2,3}_final/README.md,
experiments/2026-10-24_s95_return/README.md, the figure index and the
spec (docs/prompt-maitre-projet-uav.md, sections 4, 6.3, 6.4, 7, 10).
"""

FIGS = 'experiments/2026-10-24_thesis_figures/'
W1 = 'experiments/2026-10-23_campaign_w1_final/figures/'
W2 = 'experiments/2026-10-23_campaign_w2_final/figures/'
W3 = 'experiments/2026-10-23_campaign_w3_final/figures/'
S95 = 'experiments/2026-10-24_s95_return/figures/'
NEW = 'experiments/2026-10-31_report_analyses/'

BLOCKS = [
    # ------------------------------------------------------------ 5
    ('h1', '5 Implementation'),
    ('h2', '5.1 Software Architecture'),
    ('p',
     'The system runs on ROS 2 Jazzy with PX4 SITL (v1.17.0-alpha1) and '
     'Gazebo Harmonic (8.11.0) under Ubuntu 24.04 on WSL2 (Macenski et al., '
     '2022; Meier et al., 2015; Koenig & Howard, 2004). {fig:arch} shows '
     'what runs and the topics between the components. The code separates '
     'pure Python modules, which contain every algorithm of Section 4 and '
     'import no ROS, from four thin ROS nodes that only move messages: '
     '`cloud_world` (scan to map), `energy_monitor` (rotor power to '
     'energy), `exploration_manager` (the state machine and the only '
     'decision authority) and `trajectory_executor` (path to PX4 '
     'setpoints). The separation makes the decision logic testable '
     'without a simulator: 120 unit tests, run on every build, cover every '
     'module of Section 4 from the footprint element to the state machine, '
     'including the rules that enforce I3, I4 and I5, and every rule added '
     'after a flight came with its test. The map fusion and the terrain '
     'descriptors are validated against ground truth instead (Section '
     '6.2), and the ROS nodes and the PX4 interface by the flights.'),
    ('fig', 'arch', FIGS + 'f1_architecture.png', 16.0,
     'Software architecture as it runs: simulation and bridges (left), the '
     'four ROS 2 nodes (middle), the exploration manager and the pure '
     'modules it calls (right). Arrows carry the real topic names, PX4 ones '
     'without their `/fmu` prefixes and version suffixes; the RViz point '
     'cloud and the external stop topic are omitted. Appendix C explains '
     'the ROS 2 vocabulary and compares the figure with the graph ROS '
     'reports in flight.'),
    ('h2', '5.2 Frames and Timing'),
    ('p',
     'Four frames are involved ({fig:frames}). LiDAR points arrive in the '
     'sensor frame (forward, left, up), are converted to the body frame '
     '(forward, right, down) with the 0.15 m sensor offset, rotated into '
     'the local north-east-down frame by the attitude quaternion and '
     'translated by the vehicle position; without the rotation a point '
     '15 m ahead is misplaced in height as soon as the vehicle tilts. The '
     'simulator world, in which the ground truth is drawn, is '
     'east-north-up.'),
    ('fig', 'frames', FIGS + 'f2_frames.png', 16.0,
     'The four frames and the conversions applied by the code, for a '
     'heading of 30°: (1) LiDAR to body; (2) body to world, at the scan '
     'time; (3) simulator to world, for the ground truth.'),
    ('p',
     'Timing mattered as much as geometry. On the ramps of W1 the map was '
     'offset by one cell along the direction of flight, because the pose '
     'used to place a scan was 0.11 to 0.12 s ahead of the scene the LiDAR '
     'had seen. The fix takes the position at the scan timestamp minus a '
     'calibrated 0.060 s and the attitude last received; on six fresh '
     'surveys the offset vanished, the ramp error fell from 0.34 to '
     '0.16 m RMS and the dangerous false accepts from 3 or 4 to 0, with '
     'the recall kept. Offboard setpoints must likewise be stamped in '
     'microseconds since the autopilot booted: copying the 1 Hz '
     'synchronisation stamp caused 177 failsafes in one test series.'),
    ('h2', '5.3 Vehicle, Sensor and Flight'),
    ('p',
     'The vehicle is the PX4 x500 quadrotor with a LiDAR mounted 0.15 m '
     'below the body, so that the sensor does not see the arms; '
     '{tab:params} lists the parameters. The trajectory executor follows '
     'straight legs on the calibrated profile. Paths are planned by A\\* '
     '(Hart et al., 1968) '
     'on an obstacle grid in which a cell is blocked when its height '
     'exceeds the cruise altitude minus a 2 m vertical margin, inflated by '
     '1.25 m (half the span plus the measured tracking error).'),
    ('tab', 'params', 'Parameters of the implementation used in every '
     'experiment.',
     ['Group', 'Parameter', 'Value'],
     [['LiDAR', 'beams; rate; range; noise', '360 × 32, downward '
       'hemisphere; 10 Hz; 0.3 to 25 m (20 m used); σ = 0.02 m'],
      ['Flight', 'altitude; speed; acceleration', '8 m; 5 m/s; 3 m/s^{2}, '
       'stop at every vertex'],
      ['Map', 'cell; fusion; variance floor', '1 m; scalar Kalman, '
       'σ = 0.02 + 0.009 *d*; (0.02 m)^{2}'],
      ['Observation', 'OBSERVED', '≥ 10 returns and variance ≤ 2 × '
       '10^{-3} m^{2}'],
      ['Terrain', 'slope; roughness; step; window', '6°; 0.05 m; 0.24 m; '
       '3 × 3 plane fit'],
      ['Landing', 'footprint; confirmation', '1.5 m (3 × 3 cells); two '
       'passes per cell (*r*_{in} 14.4 m, *r*_{out} 18 m)'],
      ['Planning', 'inflation; vertical margin; escape', '1.25 m; 2 m; '
       '1 m (returns only)'],
      ['Energy', 'reserve; *E*_{land}; latency τ', '10 % of the budget; '
       '3387 J; measured, max of last five plus largest rise'],
      ['Frontiers', 'minimum cluster; viewpoint radius; blacklist',
       '3 cells; 5 m; 30 s, then 300 s after three failures'],
      ['Returns', 'switch threshold; switches', '541 J; at most 3 per '
       'return']],
     [2.6, 5.0, 8.4]),
    ('h2', '5.4 Energy Measurement, Logging and Reproducibility'),
    ('p',
     'The `energy_monitor` node integrates the rotor shaft power at 10 Hz '
     'against the logical budget of the mission. Every mission writes its '
     'configuration and git commit, the trajectory, the registry history, '
     'map snapshots every 10 s, every event, and, at every cycle, the '
     'margin and all candidates with *G*, *M*, *E*_{go}, *E*_{ret}, *U* '
     'and their admissibility, not only the winner. Campaigns are run '
     'headless: because the energy is integrated on the system clock, a '
     'simulation slowed by rendering would be charged more energy for the '
     'same flight, which is conservative but would bias the comparisons.'),
    ('p',
     '**Reproducibility.** The code, the worlds, the mission logs of every '
     'campaign reported here (about 2 GB, map snapshots included) and the '
     'scripts that draw every figure and compute every number of Sections '
     '4 to 6 are in the project repository [REPOSITORY LINK], at the tag '
     '`thesis-2026-10-27` for the flight code and the experiment '
     'directories named in each script header for the analyses. No figure '
     'needs a simulation to be redrawn.'),
    ('p',
     '**Demonstration videos.** [Video 1](https://youtu.be/ekzL6OfNzm8) '
     'presents nine filmed '
     'missions on the three worlds in RViz, each beside its energy chart; '
     '[Video 2](https://youtu.be/npNtkhwtkUM) follows one W1 mission in '
     'Gazebo itself, the camera tracking the vehicle. Appendix B lists '
     'their chapters and the section each illustrates. Rendering slows '
     'the simulation, so filmed missions illustrate the behaviour and are '
     'never used as measurements.'),

    # ------------------------------------------------------------ 6
    ('h1', '6 Experiments and Results'),
    ('h2', '6.1 Experimental Setup'),
    ('p',
     '**Worlds.** Three worlds were built, each with a ground truth '
     'rasterised from the same geometry as the simulated world, so that '
     'map and truth cannot diverge ({fig:worlds}, {tab:worlds}). W1 holds '
     'flat ground and four test zones, each designed to pass or fail one '
     'test. W2 is a field sloped at 20° with three flat pads of 16 m, two '
     'of them 62.5 m from the central one, so that safe zones are far '
     'apart. W3 mixes a road, a town with flat and gabled roofs, cars, a '
     'forest, scrub, rocks and hills, so that landable ground lies between '
     'obstacles. Each budget binds: about half of what complete coverage '
     'requires (97 kJ on W2), since a budget that fits a full exploration '
     'lets no configuration fail; on W1, 16 kJ is the budget at which a '
     'sweep showed α to act.'),
    ('fig', 'worlds', NEW + 'r4_worlds_grid.png', 16.0,
     'The three test worlds. Left: Gazebo render. Middle: ground truth, '
     'with the ground landable by design, the explored area (dashed) and '
     'the take-off point. Right: final map of a typical mission (B3, '
     'α = 0.75, coverage closest to its median), replayed through the '
     'flight perception code. Middle and right share one scale; north is '
     'up. [Video 2](https://youtu.be/npNtkhwtkUM) shows W1 in flight.'),
    ('tab', 'worlds', 'Test worlds and budgets.',
     ['World', 'Map / explored area', 'Budget', 'Reserve',
      'Character and purpose'],
     [['W1, validation', '100 m / 60 m', '16 kJ', '1.6 kJ', 'known test '
       'zones; perception against exact truth; budget where α acts'],
      ['W2, sparse pads', '200 m / 120 m', '48 kJ', '4.8 kJ', 'three flat '
       'pads in a 20° field; puts C1 under tension'],
      ['W3, mixed', '240 m / 160 m', '80 kJ', '8.0 kJ', 'town, forest, '
       'hills; zones between obstacles; registry dynamics']],
     [2.8, 3.0, 1.6, 1.6, 7.0]),
    ('p',
     '**Configurations.** B0 explores the nearest frontier with no energy '
     'reasoning until its budget is exhausted. B1 explores the frontier '
     'of largest information gain and returns to the nearest zone when '
     'its energy falls below 30 % of the budget, the classical practice. '
     'B3 applies C1 and C2 with α ∈ {0, 0.25, 0.5, 0.75, 1}; the proposal\'s '
     'C1-only configuration is B3 at α = 1 by construction. Seven '
     'configurations, ten missions each per world, give 210 missions.'),
    ('p',
     '**Protocol.** Every mission starts on a fresh simulator instance, '
     'configurations are interleaved, and the host clock is watched during '
     'every mission, after a clock fault of the WSL2 virtual machine was '
     'found to corrupt earlier flights. All 210 missions completed at the '
     'first attempt, with no technical failure, no autopilot failsafe and '
     'no clock step. The acceptance criteria and the statistical '
     'comparisons were written down before each campaign.'),
    ('p',
     '**Metrics and statistics.** Safety is measured by the number of '
     'cycles violating I1 and the minimum margin of each mission, by the '
     'use of the reserve, and by where the vehicle landed relative to the '
     'truth. Exploration is measured by coverage, the fraction of the '
     'explored area whose cells reached the OBSERVED state, and by cells '
     'observed per kilojoule. Because the purpose of the mission is to '
     'find landing zones, it is also measured by the confirmed landing '
     'area the mission leaves behind and by the energy spent until the '
     'first discovered zone is confirmed. Perception is measured by the '
     'elevation error per zone, the recall of landable cells, and the '
     'number of *dangerous* false accepts, cells declared landable whose '
     'true slope or step exceeds twice its limit. Results are medians with '
     'interquartile ranges. Because a simulated mission is not repeatable '
     'to the seed, configurations are compared with the '
     'unpaired Mann-Whitney U test (Mann & Whitney, 1947) with the '
     'rank-biserial correlation *r* as effect size, and trends in α with '
     'Spearman\'s rank correlation; comparisons beyond those fixed before '
     'the campaigns are Holm-corrected within each world (Holm, 1979).'),

    ('h2', '6.2 Perception Against Ground Truth'),
    ('p',
     'The perception results are measured on the reference map of W1, a '
     'survey map that every figure of this section uses and that was never '
     'regenerated (W1 is the top row of {fig:worlds}). {fig:mapvstruth} '
     'compares the map with the '
     'truth. On flat ground the elevation error is 0.240 m RMS with a bias '
     'of 0.014 m after removal of the constant offset of the estimator '
     'origin; the 3° ramp is mapped to 0.128 m RMS and the 10° ramp to '
     '0.466 m, its error concentrated on its raised edges. The cube is '
     'mapped 1.67 m too low on average: a single Gaussian per cell cannot '
     'represent both the top of a structure and the ground beside it, so '
     'the map shrinks raised structures at their edges. This limitation of '
     'the representation is not corrected here; for flight it is bounded '
     'by the obstacle inflation, for landing it is not, as shown below.'),
    ('fig', 'mapvstruth', FIGS + 'f4_map_vs_truth.png', 16.0,
     'Reference map of W1 against the truth: map height, true height and '
     'their difference (clipped at ±0.5 m), with the error per zone.'),
    ('p',
     '{fig:slope} justifies the slope threshold. On flat ground the slope '
     'read on the map has a 95th percentile of 4.34°, on the 10° ramp a '
     '1st percentile of 8.25°, so 6° separates the two for 95 % and 99 % '
     'of their cells; only 1 % of the 3° ramp reads above 6°. The tail of '
     'the flat '
     'distribution (99th percentile 21.66°) is not noise: 81 of the 88 '
     'flat cells that read above 6° lie within 3 m of a real step of '
     'another zone, which the representation smears beyond the one-cell '
     'margin of the analysis window. {fig:sloperough} verifies '
     'the separation of descriptors: the zone medians fall where they '
     'should, slope without roughness on the ramps (2.9° and 10.4° with '
     'roughness 0.013 and 0.014 m) and roughness without excessive slope '
     'on the rough patch (0.085 m against 0.150 m in the truth, 4.4°).'),
    ('fig', 'slope', FIGS + 'f5_slope_noise.png', 15.0,
     'Slope read on the map per zone of W1, with the 6° threshold.'),
    ('fig', 'sloperough', FIGS + 'f6_slope_roughness.png', 12.5,
     'Slope against roughness, one point per cell (log scales), with the '
     'thresholds and the zone medians (large markers): ramps read as '
     'slope, the rough patch as roughness.'),
    ('p',
     '{fig:zones} brings the chain together. With the 1.5 m footprint the '
     'detector recovers 94.2 % of the truly landable cells. It produces '
     '46 false accepts, all benign (cells slightly beyond a limit, such '
     'as a gentle slope just over 6°), and no dangerous one. But one map '
     'is an observation, not a result, so the detector was replayed on the '
     'last map of every mission of three W1 campaigns ({tab:guard}). On '
     'the first, 14 of 80 maps carried a dangerous landable cell, all in '
     'small islands of the rough patch, where the map reads a 0.12 to '
     '0.19 m step for a true 0.52 m. The confirmation guard of Section 4.4 '
     'was chosen on that campaign and held on a second, independent one, '
     'with the recall unchanged. On the final campaign, flown after the '
     'timing fix of Section 5.2, it no longer suffices: 10 of 70 maps '
     'carry a dangerous landable cell, always one or both of two cells at '
     'the raised north corner of the 3° ramp, where a true 0.66 to 0.71 m '
     'step is read as 0.04 to 0.14 m. These cells belong to the large flat '
     'zone, and 10 of the 12 lay in a confirmed zone at the end of their '
     'mission, so they were possible landing targets. No mission landed '
     'there: of the 150 C1 landings, 149 were on ground landable by design '
     'and one on forest floor, none on a dangerous cell. The absence of '
     'dangerous false accepts is therefore a property of the reference '
     'map, not of the chain: a raised edge smoothed by the single-height '
     'map remains a residual hazard on mission maps.'),
    ('tab', 'guard', 'Dangerous landable cells on the last map of every '
     'mission of three W1 campaigns, footprint 1.5 m, before and after '
     'the confirmation guard (a zone must hold a whole footprint of '
     'landable cells).',
     ['Campaign', 'Maps', 'Without guard: maps (cells)',
      'With guard: maps (cells)', 'Where'],
     [['19 Oct (guard chosen)', '80', '14 (14)', '0 (0)',
       'islands in the rough patch'],
      ['20 Oct (guard checked)', '80', '17 (22)', '0 (0)',
       'islands of fewer than 9 cells'],
      ['23 Oct, final', '70', '11 (13)', '10 (12)',
       'north corner of the 3° ramp']],
     [3.6, 1.4, 3.6, 3.4, 4.0]),
    ('fig', 'zones', FIGS + 'f7_zones_vs_truth.png', 14.5,
     'Landing cells detected on the reference map against the truth '
     '(footprint 1.5 m): recall 94.2 %, 46 benign false accepts, no '
     'dangerous one.'),

    ('h2', '6.3 Energy Prediction and Landing Precision'),
    ('p',
     '{fig:energy} shows the calibration of Section 4.5: predicted and '
     'measured energies lie on the identity line for hover segments, '
     'transits and the four validation flights, and the leg term removes '
     'the structure of the transit error by leg length. Predicted from the '
     'plan alone, the validation flights fall within 0.13 to 0.26 % of the '
     'measurement, slightly below it.'),
    ('fig', 'energy', FIGS + 'f8_energy_calibration.png', 16.0,
     'Energy model calibration. Left: predicted against measured energy '
     'per segment and for the four validation flights. Right: transit '
     'error by leg length without and with the leg term.'),
    ('p',
     'The guarantee rests on the energies C1 actually charged in missions. '
     '{fig:analyses}a compares them with the energy measured over the same '
     'interval, for every transit to a chosen frontier (2535) and every '
     'final return leg (180) of the B1 and B3 missions. Charged as C1 '
     'charges it, path plus the measured latency τ, *E*_{go} was above the '
     'measurement in 2530 transits, by 309 J at the median (14 %), and '
     'below it in 5, by 9 to 137 J; the path alone would have been short '
     'by about one second of hover, the time τ is there to cover. '
     '*E*_{ret} plus τ was above the measured return in 174 of 180 legs; '
     'five of the six exceptions are legs of 4 m or less, where stopping '
     'and detecting the arrival costs more than τ, by at most 661 J. The '
     'planning *E*_{land} exceeded the measured landing in 176 of 180 '
     'cases, by at most 10 J the other way. Return and landing together '
     'were under-charged in 5 of 180 cases, by at most 488 J against a '
     'reserve of at least 1.6 kJ.'),
    ('fig', 'analyses', NEW + 'r7_analyses.png', 16.0,
     'Four analyses of the 210 campaign missions. (a) Energy measured '
     'minus energy charged by C1, per transit (*E*_{go}) and per final '
     'return (*E*_{ret}), latency included; negative is conservative. '
     '(b) Share of frontiers rejected by C1 against the share of the '
     'budget spent. (c) Decision cycle time against observed cells. '
     '(d) Confirmed landing area at the end of the mission against α.'),
    ('p',
     'The footprint depends on how precisely the vehicle lands. Over 15 '
     'landings commanded on the same setpoint, the radial error of the '
     'touchdown was 0.124 m on average and 0.195 m at most '
     '({fig:landing}). With the blade tips 0.386 m from the centre, the '
     '1.5 m footprint leaves room for an error of 0.364 m, twice the '
     'measured 95th percentile.'),
    ('fig', 'landing', FIGS + 'f14_landing_precision.png', 15.0,
     'Landing precision. Left: radial error of 15 landings against the '
     'error each footprint can absorb. Right: the same from above, the '
     'airframe drawn at the largest error.'),

    ('h2', '6.4 Safety: the Invariant Against the Baselines'),
    ('p',
     '{tab:safety} gives the central result. No mission with C1 violated '
     'the invariant on any world, 0 of 150, and none used its reserve. B0, '
     'which ignores energy, violated it in every mission and ran out of '
     'energy in flight in all 30: on a real battery each would have ended '
     'in a crash (Video 1, [1:41](https://youtu.be/ekzL6OfNzm8?t=101)). The '
     'simulator landed them 12 to 18 s later, three times on a cell '
     'dangerous in the truth, only because the budget is a counter in the '
     'manager and not the simulated battery. Against B0 the difference in '
     'violations is '
     'significant on every world (p ≤ 0.0001, *r* = −1.00).'),
    ('tab', 'safety', 'Invariant I1 per world and configuration, ten '
     'missions per cell (B3 pools its five values of α, 50 missions).',
     ['World', 'Configuration', 'Missions violating I1',
      'Out of energy in flight', 'Energy left at landing (median)'],
     [['W1, 16 kJ', 'B0', '10/10 (median 24 cycles)', '10',
       '−4.17 kJ (budget overdrawn)'],
      ['', 'B1', '10/10 (median 6 cycles)', '0', '0.52 kJ (1.08 kJ into '
       'the 1.6 kJ reserve)'],
      ['', 'B3, all α', '**0/50**', '0', '3.1 to 3.9 kJ'],
      ['W2, 48 kJ', 'B0', '10/10 (median 44.5 cycles)', '10', '−4.05 kJ '
       '(budget overdrawn)'],
      ['', 'B1', '0/10', '0', '8.7 kJ'],
      ['', 'B3, all α', '**0/50**', '0', '6.4 to 7.0 kJ'],
      ['W3, 80 kJ', 'B0', '10/10 (median 52.5 cycles)', '10', '−4.20 kJ '
       '(budget overdrawn)'],
      ['', 'B1', '1/10 (9 cycles)', '0', '17.6 kJ'],
      ['', 'B3, all α', '**0/50**', '0', '9.6 to 11.0 kJ']],
     [2.2, 2.8, 3.8, 2.4, 4.8]),
    ('p',
     'B1 is the more instructive comparison. Its outcome depends on the '
     'pairing of its threshold with the world and the budget. On W1 at '
     '16 kJ, 30 % of the budget is less than the return, the landing and '
     'the reserve together, and B1 violated I1 in every mission, landing '
     '1.08 kJ into its reserve each time ({fig:chrono}; Video 1, '
     '[2:55](https://youtu.be/ekzL6OfNzm8?t=175)). On W2 the same '
     'threshold covers the return from anywhere in the area, and B1 held '
     'I1 in all ten missions; on W3 it failed once. A threshold can be '
     'tuned to one world; a guarantee computed from the actual registry '
     'and position needs no tuning.'),
    ('fig', 'chrono', W1 + 'f9_f10_chronogram.png', 15.0,
     'One flight on W1 at 16 kJ: energy left against what the return, '
     'the landing and the reserve need. Top: B1 violates I1 (shaded) '
     'before its 30 % return. Bottom: B3 at α = 0.75 returns when C1 '
     'admits no frontier; the margin stays positive.'),
    ('p',
     '{fig:analyses}b shows when the filter acts. Over the 2573 decisions '
     'of the C1 missions, C1 rejected no frontier while less than 30 % of '
     'the budget was spent; between 30 and 60 % it rejected some at 11 to '
     '13 % of the decisions, between 60 and 70 % at 63 %, and beyond 70 % '
     'at every decision, a median of 83 % and then 100 % of the '
     'frontiers. The filter is idle for most of a mission and decisive in '
     'its last third, where the baselines fail. The admissibility of every '
     'candidate is also computed, unused, in the baseline missions: C1 '
     'would have refused 10 of the 40 frontier choices of B1 on W1, 6 of '
     '105 on W2 and 2 of 178 on W3, a rate that follows the failures of '
     'B1 in {tab:safety}, and 66 %, 34 % and 22 % of the choices of B0.'),
    ('p',
     'The guarantee costs coverage only relative to policies that break '
     'it. On W1, B1 covered 0.89 of the area against 0.69 for B3 at '
     'α = 0.5, by taking energy from its reserve. On W2 and W3, where B1 '
     'does not overspend, C1 with C2 covered more than B1 (0.70 against '
     '0.54 on W2, 0.63 against 0.50 coverage seen on W3 at α = 0.5; '
     'p = 0.0002, *r* = +1.00 on both) and landed with less energy left: '
     'the margin that B1 wastes on a conservative threshold, C1 spends on '
     'exploration. B0 covers the most, by flying until its budget is '
     'exhausted.'),

    ('h2', '6.5 The Coverage and Margin Trade-off'),
    ('p',
     '{tab:alpha} and {fig:metrics} report the effect of α on coverage. '
     'The comparison fixed in advance, α = 0.75 against each end of the '
     'range, holds on all three worlds: 0.77 against 0.65 (α = 0) and '
     '0.76 (α = 1) on W1, 0.69 against 0.61 and 0.59 on W2, 0.60 against '
     '0.48 and 0.52 on W3 (Holm-corrected p ≤ 0.0091 in all six cases). '
     'The location of the optimum is less sharp than that. On W1, α = 0.75 '
     'covers more than every other value, significantly so against 0, '
     '0.25 and 0.5 (Holm-corrected p = 0.0018), not against α = 1 '
     '(p = 0.055). On W2 and W3 the highest median is at α = 0.5 (0.70 and '
     '0.60), and the three interior values are not distinguishable from '
     'one another after correction (α = 0.5 against 0.75: uncorrected '
     'p = 0.47 and 0.21); α = 0.5 and 0.75 cover significantly more than '
     'either end on both worlds. The supported statement is an interior '
     'optimum between α = 0.5 and 0.75, a plateau on the two larger '
     'worlds rather than a single best value.'),
    ('tab', 'alpha', 'Coverage (fraction of the explored area OBSERVED), '
     'median with interquartile range, ten missions each. Bold: the '
     'highest median of each world and the values not distinguishable '
     'from it among the B3 configurations (Mann-Whitney, Holm-corrected '
     'p > 0.05).',
     ['Configuration', 'W1, 16 kJ', 'W2, 48 kJ', 'W3, 80 kJ'],
     [['B0, nearest frontier', '0.87 [0.86, 0.88]', '0.78 [0.73, 0.80]',
       '0.60 [0.56, 0.62]'],
      ['B1, gain, 30 % return', '0.89 [0.89, 0.90]', '0.54 [0.48, 0.56]',
       '0.47 [0.43, 0.49]'],
      ['B3, α = 0', '0.65 [0.64, 0.66]', '0.61 [0.61, 0.62]',
       '0.48 [0.44, 0.49]'],
      ['B3, α = 0.25', '0.67 [0.66, 0.67]', '**0.68 [0.67, 0.69]**',
       '**0.56 [0.53, 0.62]**'],
      ['B3, α = 0.5', '0.69 [0.67, 0.71]', '**0.70 [0.67, 0.72]**',
       '**0.60 [0.60, 0.63]**'],
      ['B3, α = 0.75', '**0.77 [0.77, 0.78]**', '**0.69 [0.67, 0.70]**',
       '**0.60 [0.58, 0.60]**'],
      ['B3, α = 1', '**0.76 [0.67, 0.77]**', '0.59 [0.57, 0.61]',
       '0.52 [0.47, 0.56]']],
     [4.6, 3.8, 3.8, 3.8]),
    ('p',
     'The mechanism is visible in the decisions. The number of frontiers '
     'chosen per mission falls with α (on W3 from 40 to 21; ρ = −0.96). '
     '{fig:alpha} shows it on two W2 flights filmed for Video 1 '
     '([6:52](https://youtu.be/ekzL6OfNzm8?t=412)). At α = 0.1 the vehicle '
     'circles its take-off pad in widening loops, 19 short trips around '
     'its only confirmed zone; at α = 0.9 it strikes out on long legs to '
     'the far side, 13 trips. Both cover about the same area (0.61 and '
     '0.63); what differs is the order. At α = 1 the vehicle goes for the '
     'largest gain first, often far away, and C1 then finds no second '
     'admissible round trip: the monitor abandoned the target and ended '
     '15 of the 30 missions at α = 1, against 3 to 7 at the other values. '
     'The intermediate weights see new ground while keeping the next trips '
     'admissible. On W1 the minimum margin also decreases with α '
     '(ρ = −0.36, p = 0.01): information is bought with margin. '
     '{fig:traj} shows the same pattern over all the W2 campaign flights.'),
    ('fig', 'alpha', NEW + 'r8_alpha_w2.png', 16.0,
     'Final maps of two W2 missions at α = 0.1 and α = 0.9, filmed for '
     'Video 1; the track goes from light (start) to dark (end). '
     'Illustrations only: no campaign flew these two values.'),
    ('p',
     'C2 does not degenerate into a switch. With two admissible frontiers '
     'the per-decision rescaling maps *G* and *M* to {0, 1} and *U* can '
     'only be α or 1 − α; this happened in at most 1 % of the decisions, '
     'and 71 to 99 % had three or more admissible frontiers. Recomputed '
     'from the logged *G̃* and *M̃*, the choice at α = 0.5 differs from '
     'both the pure-gain and the pure-margin choice in 37 %, 68 % and '
     '71 % of the decisions on W1, W2 and W3. But the coverage optimum '
     'does not carry over to the quantity the mission is for. The '
     'confirmed landing area left at the end of the mission rises with α '
     '({fig:analyses}d): on W3 from '
     '328 m^{2} at α = 0 to 625 m^{2} at α = 1 (median; Spearman '
     'ρ = +0.44, p = 0.0014), on W1 from 122 to 257 m^{2} (ρ = +0.79), and '
     'it is nearly flat on W2 (93 to 98 m^{2}), where 48 of the 50 C1 '
     'missions confirmed only the central pad. The energy spent until the '
     'first '
     'discovered zone is confirmed shows no trend with α on W1 or W3 '
     '(p = 0.10 and 0.26), and on W2 it is lowest at α = 0. C2 therefore '
     'does not find zones faster; more weight on information finds more '
     'of them.'),
    ('p',
     'The mechanism is the second pass that confirmation requires. '
     'Attributing every increase of the confirmed area to the state of the '
     'vehicle, the share gained while flying to frontiers rises from α = 0 '
     'to α = 1, from 31 % to 91 % on W1, 26 % to 87 % on W2 and 81 % to '
     '93 % on W3: long excursions re-cross ground seen on the way out, '
     'whereas at low α much of the area (up to 64 % on W1 and 77 % on W2) '
     'is only confirmed during the final return. The choice of α is thus '
     'a choice of objective, 0.5 to 0.75 for coverage and higher for '
     'landing area, both inside the guarantee. B1 left a comparable area '
     'on W3 (515 m^{2}) and more on W1 (406 m^{2}), there by spending its '
     'reserve.'),
    ('fig', 'metrics', W3 + 'f11_metrics.png', 15.5,
     'Mission metrics on W3, one dot per mission, medians as bars. The '
     'α = 1 mission at 22 % coverage was ended early by an escape '
     '(Section 7).'),
    ('fig', 'traj', W2 + 'f12_trajectories.png', 16.0,
     'Every flight of the W2 campaign over the truth (flat pads in white), '
     'with the landings and the first violation of I1. Only B0 violates '
     'I1 and lands off the pads.'),

    ('h2', '6.6 Registry Dynamics and the Loss of a Zone in Use'),
    ('p',
     'The registry is not static in practice. {fig:film} follows one W3 '
     'mission from take-off to landing: the registry grows to five zones '
     'and 579 m^{2} of confirmed landing area, loses zones four times, '
     'and the energy margin narrows until C1 abandons the last target. '
     'Over the 50 C1 missions of W3 the registry shrank 90 times without '
     'any intervention; 35 of these removed the zone the vehicle was '
     'relying on and 8 the target of a return. Each was answered at the '
     'same cycle and none was followed by a violation. On W2 the registry '
     'shrank twice, and on W1 never. Replaying the perception code on the '
     'map snapshots around each W3 event, 609 of the 637 cells newly '
     'refused failed the roughness test while their variance did not '
     'change; the truth under them is flat, and they lie a median 3 m from '
     'a building, tree, car or rock. The most probable mechanism, inferred '
     'since point clouds are not logged, is the single-height map seen '
     'obliquely: returns from the side of an obstacle raise the roughness '
     'of the cells at its foot. The refusals are false but conservative; '
     'in the W3 mission of Video 1 ([0:24](https://youtu.be/ekzL6OfNzm8'
     '?t=24)) they appear as landable cells turning red at the foot of '
     'buildings and trees.'),
    ('fig', 'film', NEW + 'r5_mission_film.png', 16.0,
     'One W3 mission (B3, α = 0.75, median coverage). Top: five map '
     'snapshots with the frontiers admitted or rejected by C1, the target '
     'chosen by C2 and the confirmed zones; in (e) C1 has abandoned the '
     'target. Middle: energy left against what the return, the landing '
     'and the reserve need. Bottom: confirmed landing area, with every '
     'confirmation (green) and loss of a zone (red).'),
    ('p',
     'How often did a decision depend on a single zone? The manager also '
     'logged whether each target would have stayed admissible through the '
     'next-best confirmed zone. Of the 2423 decisions of the C1 missions, '
     '713 (29 %) relied on the provisional take-off zone, for which this '
     'was not computed; of the other 1710, 1640 (96 %) would have stayed '
     'admissible without their zone and 70 (4 %) depended on it alone. A '
     'guarantee independent of the zone in use would thus have constrained '
     'few frontier decisions; it is the return that is left without a '
     'second zone, as the provoked experiment shows.'),
    ('p',
     'To test the reaction of Section 4.4 deliberately, a field of 36 '
     'crates (2 m cubes on a 3.5 m pitch over a 20 m square, with gaps too '
     'narrow for a footprint) was placed by script over the target of a '
     'return while the vehicle was 35 to 20 m from it, before the '
     'commitment to land, on W3 at α = 1, the configuration that returns '
     'to small and distant zones most often. An earlier '
     'scenario that placed the obstacle during a frontier transit never '
     'fired in 26 missions, consistent with the 4 % above: a transit '
     'rarely relies on a single zone, whereas a return always flies '
     'towards the zone it relies on, with the least energy left.'),
    ('tab', 'provoked', 'Registry shrink provoked during a return on W3: '
     'four missions in which the crate field was placed (13 flown).',
     ['Mission', 'Zone lost (area)', 'Lost after placement',
      'Margin at re-evaluation', 'Branch', 'Landing', 'Above reserve'],
     [['r05', '16 (190 m^{2})', '3.1 s', '−278 J', 'no zone within '
       'guarantee', 'zone 4', '+0.11 kJ'],
      ['r08', '26 (174 m^{2})', '3.1 s', '+11 837 J', 'I1 holds via zone '
       '25', 'zone 4', '+12.21 kJ'],
      ['r10', '21 (209 m^{2})', '4.1 s', '−461 J', 'no zone within '
       'guarantee', 'zone 9', '0.00 kJ'],
      ['r13', '20 (49 m^{2})', '3.2 s', '−914 J', 'no zone within '
       'guarantee', 'zone 3', '−0.30 kJ']],
     [1.4, 2.6, 2.0, 2.4, 3.2, 1.8, 2.0]),
    ('p',
     '{tab:provoked} and {fig:s95} give the outcome. In all four missions '
     'the zone left the CONFIRMED state 3.1 to 4.1 s after placement, the '
     'assumed 2.5 s of detection plus at most one map cycle; I1 was '
     're-evaluated at that very cycle; the return turned to the nearest '
     'reachable zone within 0.5 s; and every landing was a committed '
     'landing on another registry zone, on ground safe in the truth. The '
     'sequence of Section 4.4 is executed as designed. The result also '
     'delimits the guarantee. In three of the four missions no zone '
     'remained within it after the loss: C1 returns when no frontier is '
     'admissible any longer, so the margin it keeps is then smaller than '
     'the detour to the next zone. The violation was logged, and the '
     'vehicle landed on the nearest zone using at most 0.30 kJ of its 8 kJ '
     'reserve. I1 holds as long as the registry does not shrink under the '
     'zone it relies on; when it does, the reserve absorbs the shortfall. '
     'Over all 188 C1 missions flown since the invariant was introduced, '
     'the reserve was touched twice (1.63 kJ before the final corrections, '
     '0.30 kJ here).'),
    ('p',
     'A fifth provoked case, flown while filming Video 1 '
     '([9:12](https://youtu.be/ekzL6OfNzm8?t=552)), exposed a gap. The '
     'crate field covered only part of the target '
     'zone, which stayed CONFIRMED through its other cells, so no loss was '
     'detected; the return flew on to its original end, inside the field, '
     'and the arrival committed the landing there: the commit checks that '
     'the vehicle has arrived, not that the cell is still landable. In the '
     'final campaigns, 149 of 150 commits were over a cell the map still '
     'held landable, the other over a refused road cell, safe in the '
     'truth. A first correction, replanning such a return, kept a filmed '
     'vehicle out of the crates but delayed landings over flickering '
     'cells and broke I1 in 2 of 5 filmed flights; it is not part of the '
     'evaluated system.'),
    ('fig', 's95', S95 + 'f13_B3a100_r13.png', 15.5,
     'Provoked loss of the zone in use during a return (W3, mission r13). '
     'Top: the map before and after the crates are seen, with the '
     'reaction and the landing on zone 3. Bottom: I1 is violated from the '
     'loss to the landing commit (shaded); the reserve covers 0.30 kJ.'),

    ('h2', '6.7 Computational Cost'),
    ('p',
     'The method runs online, one decision cycle per map at about 1 Hz. '
     '{fig:analyses}c gives the computation time of every cycle of the '
     'final campaigns: a median of 29 ms on W1, 103 ms on W2 and 198 ms on '
     'W3, with a maximum of 664 ms on W3, within the one-second cycle on '
     'the development laptop. Time grows with the observed part of the map '
     '(ρ = +0.78) and with the number of candidates (ρ = +0.84, about '
     '2.5 ms per candidate on W3). The dominant terms '
     'are the distance field from the vehicle (68 ms median on W3) and the '
     'decision step (54 ms), which updates the registry, computes the '
     'return field from the confirmed zones and evaluates every candidate; '
     'both fields are recomputed at every cycle. Each field is computed by '
     'vectorised relaxation sweeps until convergence, so its cost grows '
     'with the number of cells *N* times the longest path in cells *L*. '
     'This sets the scaling limit: a 1 km^{2} area at the same resolution '
     'has 17 times the cells of W3 and paths about four times longer, so '
     'by extrapolation a field would take several seconds. The guarantee '
     'would not break, since τ charges the decision time as hover, but '
     'latency would consume budget; heap-based or incremental shortest '
     'paths would restore the margin. The 21.5 s decisions of Section 4.6 '
     'predate the rewriting of two other steps of the cycle.'),
    ('h2', '6.8 Negative Results and Discussion'),
    ('p',
     'Several results were negative, and each changed the design or its '
     'claims. The energy model and the telemetry of the proposal were '
     'refuted (Section 4.5), and a constant decision latency broke I1 '
     '(Section 4.6). A Mahalanobis gate meant to handle discontinuities '
     'raised the dangerous false accepts from 26 to 138, because it locked '
     'each cell to the first surface seen: any fix to the single-height '
     'limit must keep two surface hypotheses per cell. The invariant itself '
     'first failed on W3 (2 of 25 C1 missions, after the loss of their '
     'return target) until three corrections, each measured on its own, '
     'brought the count to 0 of 150. The confirmation '
     'guard that removed every dangerous false accept on two campaigns did '
     'not on the third (Section 6.2), and the coverage optimum of C2 is '
     'not an optimum for the landing area found (Section 6.5). And a '
     'well-tuned fixed threshold holds I1 (B1 on W2): the advantage of C1 '
     'is not to beat every threshold but to hold without tuning.'),
    ('p',
     'Taken together, the results answer the sub-questions behind the '
     'research question. The map supports a classification with a recall '
     'near 94 % and no dangerous false accept on the reference map, but '
     'not on every mission map, where one smoothed raised edge recurs. The '
     'calibrated model predicts energy within 0.3 % on unseen '
     'trajectories, and the energies C1 charged in flight were '
     'conservative in all but a handful of short, explained cases. C1 '
     'prevents the vehicle from entering a state in which no confirmed '
     'zone is reachable, as long as the registry does not shrink under the '
     'zone in use, at no coverage cost relative to a baseline that '
     'respects the same reserve; it acts in the last third of a mission. '
     'And C2 exposes a real trade-off: an interior optimum for coverage '
     'between α = 0.5 and 0.75 on all three worlds, and more landing area '
     'with more weight on information.'),

    # ------------------------------------------------------------ 7
    ('h1', '7 Conclusion and Future Work'),
    ('h2', '7.1 Conclusion'),
    ('p',
     'This report addressed how a UAV can explore unknown terrain for safe '
     'landing zones while keeping a confirmed zone continuously reachable. '
     'The answer proposed is an energy-feasibility invariant over a '
     'registry of zones discovered online, enforced as a filter on every '
     'exploration decision and monitored at every map update, together '
     'with a utility that trades information against energy margin. The '
     'system kept the invariant in all 150 missions in which it was '
     'active, across three worlds, where a coverage-only explorer broke it '
     'every time and a fixed-threshold return broke it whenever its '
     'threshold did not suit the world; the energies it charged were '
     'checked against every leg flown. The trade-off parameter has an '
     'interior optimum for coverage, between 0.5 and 0.75, while the '
     'landing area found grows with it. The deliberate removal of a zone '
     'in use showed the reaction working as designed and the scope of the '
     'guarantee: it rests on the zone the vehicle relies on, and the '
     'reserve covers the case where that zone disappears late.'),
    ('p',
     'The limitations must be stated with the same precision. All results '
     'are in simulation, with a static environment, a single sensor and '
     'the mechanical rotor energy of the simulator, which excludes motor '
     'and battery losses. The take-off point is assumed free for the whole '
     'mission, the landing is committed without checking that the end '
     'cell of the return is still landable (Section 6.6), and nothing is '
     're-evaluated after the commit. '
     'The single-height map shrinks raised structures, causes false '
     'refusals near obstacles and, on mission maps, can accept a cell at '
     'the foot of a smoothed raised edge. After C1 abandons a target, or '
     'after an escape, the vehicle returns instead of selecting again, '
     'which leaves some missions with admissible frontiers unused. And five '
     'experiments that would test these conclusions further were not run: '
     'an ablation keeping the registry and the monitor but no filter on '
     'the selection, which alone would isolate the filter of C1; '
     'randomised take-off points, which would make the ten missions per '
     'configuration a sample of the world rather than of the simulator; a '
     'sweep of the reserve *R*; injected localisation noise; and wind. '
     'Every result holds only for a clean GPS pose, still air, one '
     'take-off point per world and *R* = 10 %.'),
    ('h2', '7.2 Short-Term Future Work'),
    ('p',
     'With three more months, the first improvements would address these '
     'limits directly. (i) Run the five experiments listed above, the '
     'monitor-only ablation and the randomised take-off points first. '
     '(ii) Select again after an abort or an escape instead of returning, '
     'which would recover the coverage those missions leave. '
     '(iii) Make the end-cell check of Section 6.6 robust to flickering '
     'cells, repeat the provoked experiment, implement the landing abort '
     'when an obstacle '
     'appears on the committed site, and let observations retire the '
     'take-off point like any other zone. (iv) Replace the single Gaussian per cell by '
     'two surface hypotheses, which addresses both the oblique false '
     'refusals that drive the demotions on W3 and the smoothed edge that '
     'defeats the confirmation guard. (v) Integrate the energy on '
     'simulation time rather than system time. (vi) Offer, as an option, '
     'a stricter guarantee that holds without the zone in use and measure '
     'its coverage cost; Section 6.6 suggests it would bind mostly on the '
     'return. (vii) Replace the relaxation sweeps by a heap-based or '
     'incremental shortest-path field before scaling the area up '
     '(Section 6.7).'),
    ('h2', '7.3 Long-Term Research Directions'),
    ('p',
     'At the scale of a PhD, three directions stand out. The first is to '
     'make the guarantee robust to uncertainty: replace the deterministic '
     'energy prediction by a conservative bound, as the proposal '
     'anticipated, and state the invariant as a chance constraint over '
     'battery state, wind and map error, validated on real flights with an '
     'electrical energy model. The second is to treat landing safety as '
     'an object of active perception: confirming a zone requires a second '
     'view, so the planner could choose where to look not only for '
     'coverage but to confirm or reject candidate zones, with the value of '
     'a confirmation measured by the energy margin it unlocks. The third '
     'is to share and maintain the registry over time and across vehicles, '
     'which raises the question of how a guarantee should account for '
     'zones that other agents observed, and for a terrain that changes.'),
    ('count_off',),

    # ------------------------------------------------- declaration
    ('h1', 'Declaration of Generative AI Usage'),
    ('p',
     'This work used Claude Code (Anthropic, 2026a), Anthropic\'s coding '
     'assistant, running the large language model Claude Opus 5.5 '
     '(Anthropic, 2026b) in the author\'s development environment (Visual '
     'Studio Code with Ubuntu under WSL2), where it could read and edit '
     'the project files and run commands. This use is within the purposes '
     'permitted for the assessment. The complete chat transcripts are '
     'submitted with this report, each declared at its top as a true '
     'transcript.'),
    ('p', 'Claude was used for four tasks, on the author\'s instructions:'),
    ('ul', [
        '**Code.** It helped test, organise and partially write the code: '
        'the ROS 2 package (the four nodes and the pure Python modules of '
        'Section 5.1, with their unit tests), the experiment and campaign '
        'scripts, and the analysis scripts.',
        '**Experiments.** It ran the simulation campaigns on my own '
        'computer. Instead of my launching several hundred PX4 SITL '
        'missions by hand, Claude ran them as background jobs, monitored '
        'them and reported the results while I worked on other tasks. It '
        'then ran the analyses and drew the figures from the mission logs, '
        'with scripts kept in the repository.',
        '**Writing.** It drafted the report and some figures using the '
        'results of the campaigns (hundreds of simulations), my outline '
        'and instructions, and revised them after each of my reviews.',
        '**Video.** It edited the two demonstration videos (cuts, title '
        'cards, energy charts and assembly) from screen recordings of '
        'simulated missions made beforehand.']),
    ('p',
     'Claude proposed many technical choices, each documented with its '
     'reason in the code and the experiment notes; I reviewed them '
     'carefully, accepted, changed or rejected them, and can explain and '
     'defend every part of the work.'),

    # --------------------------------------------------- references
    ('h1', 'References'),
    ('ref', 'Abeywickrama, H. V., Jayawickrama, B. A., He, Y., & '
     'Dutkiewicz, E. (2018). Comprehensive energy consumption model for '
     'unmanned aerial vehicles, based on empirical studies of battery '
     'performance. *IEEE Access, 6*, 58383–58394. '
     'https://doi.org/10.1109/ACCESS.2018.2875040'),
    ('ref', 'Ames, A. D., Coogan, S., Egerstedt, M., Notomista, G., '
     'Sreenath, K., & Tabuada, P. (2019). Control barrier functions: Theory '
     'and applications. In *2019 18th European Control Conference (ECC)* '
     '(pp. 3420–3431). IEEE. https://doi.org/10.23919/ECC.2019.8796030'),
    ('ref', 'Anthropic. (2026a). *Claude Code* (Versions 2.1.301–2.1.314) '
     '[Computer software]. https://claude.com/claude-code'),
    ('ref', 'Anthropic. (2026b). *Claude Opus 5.5* (claude-opus-5-5) '
     '[Large language model]. https://www.anthropic.com/claude'),
    ('ref', 'Beycimen, S., Ignatyev, D., & Zolotas, A. (2026). Real-time '
     'terrain traversability analysis and mapping for autonomous robotics '
     'in dynamic environments: Fusing appearance- and geometry-based '
     'approaches. *Journal of Field Robotics, 43*(3), 1719–1738. '
     'https://doi.org/10.1002/rob.70097'),
    ('ref', 'Bircher, A., Kamel, M., Alexis, K., Oleynikova, H., & '
     'Siegwart, R. (2016). Receding horizon "next-best-view" planner for 3D '
     'exploration. In *2016 IEEE International Conference on Robotics and '
     'Automation (ICRA)* (pp. 1462–1468). IEEE. '
     'https://doi.org/10.1109/ICRA.2016.7487281'),
    ('ref', 'Brubaker, K. M., Myers, W. L., Drohan, P. J., Miller, D. A., '
     '& Boyer, E. W. (2013). The use of LiDAR terrain data in '
     'characterizing surface roughness and microtopography. *Applied and '
     'Environmental Soil Science, 2013*, 1–13. '
     'https://doi.org/10.1155/2013/891534'),
    ('ref', 'Cadena, C., Carlone, L., Carrillo, H., Latif, Y., '
     'Scaramuzza, D., Neira, J., Reid, I., & Leonard, J. J. (2016). Past, '
     'present, and future of simultaneous localization and mapping: Toward '
     'the robust-perception age. *IEEE Transactions on Robotics, 32*(6), '
     '1309–1332. https://doi.org/10.1109/TRO.2016.2624754'),
    ('ref', 'Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M. '
     'M., & Tardós, J. D. (2021). ORB-SLAM3: An accurate open-source '
     'library for visual, visual-inertial, and multimap SLAM. *IEEE '
     'Transactions on Robotics, 37*(6), 1874–1890. '
     'https://doi.org/10.1109/TRO.2021.3075644'),
    ('ref', 'Cesare, K., Skeele, R., Yoo, S.-H., Zhang, Y., & Hollinger, G. '
     '(2015). Multi-UAV exploration with limited communication and '
     'battery. In *2015 IEEE International Conference on Robotics and '
     'Automation (ICRA)* (pp. 2230–2235). IEEE. '
     'https://doi.org/10.1109/ICRA.2015.7139494'),
    ('ref', 'Cherenson, D. M., Agrawal, D. R., & Panagou, D. (2025). '
     '*Autonomy architectures for safe planning in unknown environments '
     'under budget constraints* (arXiv:2504.03001). arXiv. '
     'https://doi.org/10.48550/arXiv.2504.03001'),
    ('ref', 'Elskamp, J., Shi, M., Bauersfeld, L., Scaramuzza, D., & '
     'Popović, M. (2026). *EAAE: Energy-aware autonomous exploration for '
     'UAVs in unknown 3D environments* (arXiv:2603.15604). arXiv. '
     'https://doi.org/10.48550/arXiv.2603.15604'),
    ('ref', 'Fankhauser, P., Bloesch, M., Gehring, C., Hutter, M., & '
     'Siegwart, R. (2014). Robot-centric elevation mapping with '
     'uncertainty estimates. In *Mobile Service Robotics: Proceedings of '
     'the 17th International Conference on Climbing and Walking Robots '
     '(CLAWAR 2014)* (pp. 433–440). World Scientific. '
     'https://doi.org/10.3929/ethz-a-010173654'),
    ('ref', 'Fankhauser, P., Bloesch, M., & Hutter, M. (2018). '
     'Probabilistic terrain mapping for mobile robots with uncertain '
     'localization. *IEEE Robotics and Automation Letters, 3*(4), '
     '3019–3026. https://doi.org/10.1109/LRA.2018.2849506'),
    ('ref', 'Gasche, S., Kallies, C., Himmel, A., & Findeisen, R. (2025). '
     '*Energy aware and safe path planning for unmanned aircraft systems* '
     '(arXiv:2504.03271). arXiv. https://doi.org/10.48550/arXiv.2504.03271'),
    ('ref', 'Gramajo, G., & Shankar, P. (2017). An efficient energy '
     'constraint based UAV path planning for search and coverage. '
     '*International Journal of Aerospace Engineering, 2017*, 1–13. '
     'https://doi.org/10.1155/2017/8085623'),
    ('ref', 'Han, W. (2023). *Risk aware planning and probabilistic '
     'prediction for autonomous systems under uncertain environments* '
     '[Doctoral dissertation, Massachusetts Institute of Technology]. '
     'https://hdl.handle.net/1721.1/150099'),
    ('ref', 'Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal '
     'basis for the heuristic determination of minimum cost paths. *IEEE '
     'Transactions on Systems Science and Cybernetics, 4*(2), 100–107. '
     'https://doi.org/10.1109/TSSC.1968.300136'),
    ('ref', 'Holm, S. (1979). A simple sequentially rejective multiple test '
     'procedure. *Scandinavian Journal of Statistics, 6*(2), 65–70.'),
    ('ref', 'Hornung, A., Wurm, K. M., Bennewitz, M., Stachniss, C., & '
     'Burgard, W. (2013). OctoMap: An efficient probabilistic 3D mapping '
     'framework based on octrees. *Autonomous Robots, 34*(3), 189–206. '
     'https://doi.org/10.1007/s10514-012-9321-0'),
    ('ref', 'Johnson, A. E., Klumpp, A. R., Collier, J. B., & Wolf, A. A. '
     '(2002). Lidar-based hazard avoidance for safe landing on Mars. '
     '*Journal of Guidance, Control, and Dynamics, 25*(6), 1091–1099. '
     'https://doi.org/10.2514/2.4988'),
    ('ref', 'Kalman, R. E. (1960). A new approach to linear filtering and '
     'prediction problems. *Journal of Basic Engineering, 82*(1), 35–45. '
     'https://doi.org/10.1115/1.3662552'),
    ('ref', 'Koenig, N., & Howard, A. (2004). Design and use paradigms for '
     'Gazebo, an open-source multi-robot simulator. In *2004 IEEE/RSJ '
     'International Conference on Intelligent Robots and Systems (IROS)* '
     '(pp. 2149–2154). IEEE. https://doi.org/10.1109/IROS.2004.1389727'),
    ('ref', 'Liu, Z., Sengupta, R., & Kurzhanskiy, A. (2017). A power '
     'consumption model for multi-rotor small unmanned aircraft systems. '
     'In *2017 International Conference on Unmanned Aircraft Systems '
     '(ICUAS)* (pp. 310–315). IEEE. '
     'https://doi.org/10.1109/ICUAS.2017.7991310'),
    ('ref', 'Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & '
     'Woodall, W. (2022). Robot Operating System 2: Design, architecture, '
     'and uses in the wild. *Science Robotics, 7*(66), eabm6074. '
     'https://doi.org/10.1126/scirobotics.abm6074'),
    ('ref', 'Mann, H. B., & Whitney, D. R. (1947). On a test of whether one '
     'of two random variables is stochastically larger than the other. '
     '*The Annals of Mathematical Statistics, 18*(1), 50–60. '
     'https://doi.org/10.1214/aoms/1177730491'),
    ('ref', 'Meier, L., Honegger, D., & Pollefeys, M. (2015). PX4: A '
     'node-based multithreaded open source robotics framework for deeply '
     'embedded platforms. In *2015 IEEE International Conference on '
     'Robotics and Automation (ICRA)* (pp. 6235–6240). IEEE. '
     'https://doi.org/10.1109/ICRA.2015.7140074'),
    ('ref', 'Papachristos, C., Khattak, S., & Alexis, K. (2017). '
     'Uncertainty-aware receding horizon exploration and mapping using '
     'aerial robots. In *2017 IEEE International Conference on Robotics and '
     'Automation (ICRA)* (pp. 4568–4575). IEEE. '
     'https://doi.org/10.1109/ICRA.2017.7989531'),
    ('ref', 'Scherer, S., Chamberlain, L., & Singh, S. (2012). Autonomous '
     'landing at unprepared sites by a full-scale helicopter. *Robotics '
     'and Autonomous Systems, 60*(12), 1545–1562. '
     'https://doi.org/10.1016/j.robot.2012.09.004'),
    ('ref', 'Shah Alam, M., & Oluoch, J. (2021). A survey of safe landing '
     'zone detection techniques for autonomous unmanned aerial vehicles '
     '(UAVs). *Expert Systems with Applications, 179*, 115091. '
     'https://doi.org/10.1016/j.eswa.2021.115091'),
    ('ref', 'Takahashi, M. D., Abershitz, A., Rubinets, R., & Whalley, M. '
     'S. (2013). Evaluation of safe landing area determination algorithms '
     'for autonomous rotorcraft using site benchmarking. *Journal of the '
     'American Helicopter Society, 58*(3), 1–13. '
     'https://doi.org/10.4050/JAHS.58.032007'),
    ('ref', 'Yamauchi, B. (1997). A frontier-based approach for autonomous '
     'exploration. In *Proceedings of the 1997 IEEE International Symposium '
     'on Computational Intelligence in Robotics and Automation (CIRA\'97)* '
     '(pp. 146–151). IEEE. https://doi.org/10.1109/CIRA.1997.613851'),

    # ----------------------------------------------------- appendix
    ('pagebreak',),
    ('h1', 'Appendix A: Communication Log with Supervisor'),
    ('placeholder', 'Semester 2 communication log, to be added.'),

    ('pagebreak',),
    ('h1', 'Appendix B: Demonstration Videos'),
    ('p',
     'Both videos were recorded from simulated missions flown with the '
     'flight code of the campaigns. Rendering slows the simulation, and '
     'the energy is integrated on the system clock, so a filmed mission is '
     'charged more energy than a measured one: the videos illustrate the '
     'behaviour reported in Sections 6.4 to 6.6, and no number of the '
     'report comes from them.'),
    ('p',
     '**Video 1**, [https://youtu.be/ekzL6OfNzm8](https://youtu.be/'
     'ekzL6OfNzm8) (10 min 40 s). Nine missions on the three worlds, each '
     'shown in RViz, the vehicle\'s own map, beside its energy chart drawn '
     'from the mission log: energy left against what the return, the '
     'landing and the reserve need. A card introduces each mission, with '
     'an orbit view of the world when it first appears. {tab:video} gives '
     'the chapters; each time is a link to that moment.'),
    ('tab', 'video', 'Chapters of Video 1 and the results they illustrate.',
     ['Time', 'Mission', 'What it shows', 'Section'],
     [['[0:00](https://youtu.be/ekzL6OfNzm8?t=0)', 'none',
       'title and legend: map colours, zone states, energy chart', ''],
      ['[0:24](https://youtu.be/ekzL6OfNzm8?t=24)', 'W3, B3, α = 0.75',
       'a full mission: zones confirmed and lost, frontiers rejected by '
       'C1 late in the flight, return and landing on a confirmed zone',
       '6.4, 6.6'],
      ['[1:41](https://youtu.be/ekzL6OfNzm8?t=101)', 'W3, B0',
       'no energy reasoning: I1 violated, energy exhausted in flight', '6.4'],
      ['[2:55](https://youtu.be/ekzL6OfNzm8?t=175)', 'W1, B1',
       'the 30 % return comes too late on 16 kJ: landing in the reserve',
       '6.4'],
      ['[3:57](https://youtu.be/ekzL6OfNzm8?t=237)', 'W1, B3, α = 0.75',
       'same world and budget: C1 ends the exploration, the margin stays '
       'positive', '6.4'],
      ['[4:41](https://youtu.be/ekzL6OfNzm8?t=281)', 'W2, B0',
       'far from every pad when the energy runs out', '6.4'],
      ['[5:52](https://youtu.be/ekzL6OfNzm8?t=352)', 'W2, B3, α = 0.75',
       'exploration around the reachable pad, landing on it', '6.5'],
      ['[6:52](https://youtu.be/ekzL6OfNzm8?t=412)',
       'W2, α = 0.1 and 0.9', 'the two flights of {fig:alpha}, then their '
       'final maps side by side (8:56)', '6.5'],
      ['[9:12](https://youtu.be/ekzL6OfNzm8?t=552)',
       'W3, crates on the return target', 'the zone survives in part; the '
       'landing is committed among the crates', '6.6'],
      ['[10:26](https://youtu.be/ekzL6OfNzm8?t=626)', 'none',
       'summary of the measured results', '']],
     [1.6, 3.4, 9.4, 1.6]),
    ('p',
     '**Video 2**, [https://youtu.be/npNtkhwtkUM](https://youtu.be/'
     'npNtkhwtkUM) (1 min 10 s). One W1 mission (B3, α = 0.75, 16 kJ) '
     'seen in Gazebo itself, in real time, the camera following the '
     'vehicle from take-off over the ramps and the rough patch to its '
     'landing on a confirmed zone; a panel drawn from the mission log '
     'gives the mission time, the state and the energy left against what '
     'I1 needs.'),

    ('pagebreak',),
    ('h1', 'Appendix C: The ROS 2 Graph at Run Time'),
    ('p',
     'Figure 3 is drawn from the code, not captured from the running '
     'system. This appendix explains the ROS 2 vocabulary for readers who '
     'do not use it, then compares the figure with the graph that ROS '
     'itself reports during a mission.'),
    ('p',
     '**ROS 2 in brief.** ROS 2 (Macenski et al., 2022) is not an '
     'operating system but a communication framework: it lets separate '
     'programs, each with one job, exchange data, here on one computer. '
     'Five notions are enough to read Figure 3 and {fig:rqt}:'),
    ('ul', [
        '**Node**: one running program with one role, drawn as an ellipse '
        'by rqt_graph. `cloud_world`, for example, turns LiDAR scans into '
        'the elevation map. A node can be started, stopped, replaced or '
        'tested on its own.',
        '**Topic**: a named, typed stream of messages, drawn as a '
        'rectangle, such as `/map/elevation`. A node *publishes* on it '
        'without knowing who listens, and any number of nodes *subscribe* '
        'to it. It suits data produced continuously: scans, poses, energy, '
        'maps.',
        '**Service**: a request followed by one reply, like a function '
        'call to another node, for a short query.',
        '**Action**: a long task with a goal, progress reports and the '
        'possibility to cancel it, such as "fly to this point".',
        '**Parameter**: a named setting of a node, read at start-up and, '
        'if the node allows it, changeable while it runs.']),
    ('p',
     'The system uses topics and parameters only: its four nodes offer no '
     'service or action of their own. Every exchange is a stream that one '
     'node produces and others read at their own pace, which is what '
     'topics are for. Of the parameters, only α and the reserve can be '
     'changed in flight, through the parameter interface every node has '
     '(the rqt_reconfigure tool of Figure 3); the others are read at '
     'start-up. Two bridges connect programs that are not ROS nodes: '
     '`ros_gz_bridge` republishes the LiDAR points, motor speeds and '
     'clock of the Gazebo simulator, and the Micro XRCE-DDS agent carries '
     'the internal messages of the PX4 autopilot as `/fmu/...` topics.'),
    ('p',
     '**The graph reported by ROS.** rqt_graph is the standard ROS tool '
     'that draws the nodes and topics it discovers. {fig:rqt} was '
     'generated with its own code and the default filters of its window, '
     '5 s after the vehicle began its first frontier transit in a W1 '
     'mission (B3, α = 0.75, 16 kJ, without display), and laid out top to '
     'bottom to fit the page; the scripts and the full lists of '
     'publishers and subscriptions of every node are in '
     '`experiments/2026-11-01_rqt_graph`. It differs from Figure 3 in '
     'three ways, all due to what rqt_graph can see:'),
    ('ul', [
        'PX4 does not appear. Its messages enter ROS through the bridge '
        'agent, not through a ROS node, so rqt_graph draws no PX4 box. Of '
        'its topics only the vehicle position is kept, because three nodes '
        'read it; the others have a single ROS endpoint and are hidden by '
        'the default filter.',
        'Some topics are filtered out: the RViz display topics '
        '(`/viz/...`), which nobody reads when no display runs, and the '
        'simulation clock, hidden as infrastructure.',
        'The inside of the exploration manager is invisible: the registry, '
        'the energy model, C1 and C2 are Python modules called by one '
        'node, not nodes. Figure 3 draws them because they hold the '
        'contributions.']),
    ('p',
     'Every topic drawn in Figure 3 was found on the node it is drawn on. '
     'The comparison led to one correction of the figure, which now shows '
     'that the trajectory executor also reads the vehicle position, to '
     'detect its arrival. The figure still omits two topics that play no '
     'part in the method: the point cloud published for RViz and the '
     'external stop used by the campaign scripts.'),
    ('fig', 'rqt', 'experiments/2026-11-01_rqt_graph/rqt_graph_w1.png',
     15.0,
     'The ROS graph during a W1 mission, as rqt_graph draws it in its '
     '"Nodes/Topics (active)" view with default filters: nodes as '
     'ellipses, topics as rectangles grouped by namespace, arrows from '
     'publisher to subscriber.'),
]
