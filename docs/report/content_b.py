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
     'setpoints). The separation is what makes the decision logic testable '
     'without a simulator: 120 unit tests cover the map fusion, the terrain '
     'analysis, the registry, the energy model, C1, C2 and the state '
     'machine, and every rule added after a flight came with its test.'),
    ('fig', 'arch', FIGS + 'f1_architecture.png', 16.0,
     'Software architecture as it runs: Gazebo and PX4 SITL with their '
     'two bridges (left), the four ROS 2 nodes of the package (middle), '
     'and the exploration manager with the pure modules it calls (right). '
     'Arrows carry the real topic names; PX4 topics are shortened (the '
     '`/fmu/in` and `/fmu/out` prefixes and version suffixes are omitted). '
     'Dashed boxes are the contributions; solid ones are reused '
     'techniques or infrastructure.'),
    ('h2', '5.2 Frames and Timing'),
    ('p',
     'Four frames are involved ({fig:frames}). LiDAR points arrive in the '
     'sensor frame (forward, left, up) and are converted to the body frame '
     '(forward, right, down) by inverting the *y* and *z* axes and adding '
     'the 0.15 m sensor offset. They are then rotated into the local '
     'north-east-down frame by the attitude quaternion and translated by '
     'the vehicle position. Without this rotation a point 15 m ahead is '
     'placed at the wrong height as soon as the vehicle tilts, which in an '
     'earlier version of the project turned the whole terrain unsafe. The '
     'simulator world, in which the ground truth is drawn, is east-north-up '
     'and is converted by swapping the horizontal axes and negating the '
     'height.'),
    ('fig', 'frames', FIGS + 'f2_frames.png', 16.0,
     'The four frames and the conversions applied by the code, drawn for '
     'a heading 30° east of north so that the body and world frames '
     'visibly differ. (1) LiDAR to body; (2) body to world, with the '
     'position taken at the scan time; (3) simulator to world, used for '
     'the ground truth; the grid indexing in the world frame is given on '
     'the right.'),
    ('p',
     'Timing turned out to matter as much as geometry. On the ramps of the '
     'validation world the map was offset by one cell from the truth along '
     'the direction of flight. The cause was measured: the pose used to '
     'place a scan was 0.11 to 0.12 s ahead of the scene the LiDAR had '
     'seen, which at 5 m/s smears the map by half a cell. The fix takes '
     'the position at the scan timestamp minus a calibrated 0.060 s, '
     'through the simulation clock, and the attitude last received; taking '
     'the attitude back in time as well destroyed the recall offline and '
     'was rejected. On six fresh surveys the offset vanished, the ramp '
     'error fell from 0.34 to 0.16 m RMS, and the dangerous false accepts '
     'of fresh surveys fell from 3 or 4 to 0, while the recall was kept '
     '(93.7 % before, 94.4 to 95.7 % after). Several silent failure modes '
     'of the PX4 interface had to be resolved as well; the most costly was '
     'that offboard setpoints must be stamped in microseconds since the '
     'autopilot booted, extrapolated from a time offset updated at 1 Hz, '
     'and copying the 1 Hz stamp itself caused 177 failsafes in one test '
     'series, none after the fix.'),
    ('h2', '5.3 Vehicle, Sensor and Flight'),
    ('p',
     'The vehicle is the PX4 x500 quadrotor with a LiDAR mounted 0.15 m '
     'below the body, a placement chosen so that the sensor does not see '
     'the arms. {tab:params} lists the parameters used in all experiments. '
     'The trajectory executor follows a path of straight legs on the '
     'calibrated profile and declares arrival on separate horizontal and '
     'vertical tolerances. Paths are planned by A\\* (Hart et al., 1968) '
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
     'and publishes the energy left against the logical budget of the '
     'mission. Every mission writes one directory containing its full '
     'configuration and git commit, the trajectory, the registry history '
     'with every state transition, map snapshots, every evaluation of the '
     'invariant with its margin, every event, and, for every selection '
     'cycle, all candidates with *G*, *M*, *E*_{go}, *E*_{ret}, *U* and '
     'their admissibility, not only the winner. Every figure of Section 6 '
     'is regenerated by a script from these logs alone, without running '
     'the simulation again. Campaigns are run headless: rendering a large '
     'map slows the simulation, and because the energy is integrated on '
     'the system clock a slower simulation is charged more energy for the '
     'same flight, which is conservative but would bias the comparisons.'),

    # ------------------------------------------------------------ 6
    ('h1', '6 Experiments and Results'),
    ('h2', '6.1 Experimental Setup'),
    ('p',
     '**Worlds.** Three worlds were built, each with a ground truth '
     'rasterised from the same geometry as the simulated world, so that '
     'map and truth cannot diverge ({fig:worlds}, {tab:worlds}). W1, the '
     'validation world, holds flat ground and four test zones: a 3° ramp '
     'that must be accepted, a 10° ramp that must be refused, a rough '
     'patch that must be refused on roughness rather than slope, and a 4 m '
     'cube that must be refused on step. W2 is a 176 m field sloped at '
     '20° with only three flat pads of 16 m, two of them 62.5 m from the '
     'centre, so that safe zones are far apart and the margin is consumed '
     'quickly. W3 is a mixed 240 m world with a road, a town with flat '
     'and gabled roofs, cars, a forest, scrub, rocks and hills, in which '
     'landable ground lies between obstacles. The budget of each world was '
     'set so that it binds: about half of what complete coverage requires '
     '(97 kJ on W2), since if a full exploration fits comfortably in the '
     'budget no configuration can fail and the experiment proves nothing. '
     'On W1 the budget of 16 kJ was chosen from a budget sweep as the one '
     'at which α measurably changes the behaviour.'),
    ('fig', 'worlds', FIGS + 'r3_worlds.png', 16.0,
     'The three test worlds from above, from their ground truth: ground '
     'landable by design, other ground whose slope or roughness exceeds '
     'the limits, and obstacles. The dashed square is the explored area, '
     'the cross the take-off point. W2 offers three distant pads in a '
     'steep field; W3 mixes a town, a forest and open ground.'),
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
     'reasoning and lands where its budget ends. B1 explores the frontier '
     'of largest information gain and returns to the nearest zone when '
     'its energy falls below 30 % of the budget, the classical practice '
     'and the baseline that matters. B3 applies C1 and C2 with α swept over '
     '{0, 0.25, 0.5, 0.75, 1}; the configuration of the proposal with C1 '
     'and pure gain ranking is B3 at α = 1 by construction and was not '
     'flown separately. This gives seven configurations, ten missions each '
     'per world, 210 missions in all, instead of the 240 initially '
     'planned.'),
    ('p',
     '**Protocol.** Every mission starts on a fresh simulator instance '
     'with reset autopilot parameters, configurations are interleaved '
     'rather than run in blocks, and the host clock is checked before and '
     'watched during every mission, after a clock fault of the WSL2 '
     'virtual machine was found to corrupt some earlier flights. A '
     'mission that fails for a technical reason is flown again and '
     'reported, never discarded. All 210 missions of the final campaigns '
     'completed at the first attempt, with no technical failure, no '
     'autopilot failsafe and no clock step. The acceptance criteria and the '
     'statistical comparisons were written down before each campaign.'),
    ('p',
     '**Metrics and statistics.** Safety is measured by the number of '
     'cycles violating I1 and the minimum margin of each mission, by the '
     'use of the reserve, and by where the vehicle landed relative to the '
     'truth. Exploration is measured by coverage, the fraction of the '
     'explored area whose cells reached the OBSERVED state, and by cells '
     'observed per kilojoule. Perception is measured against the truth by '
     'the elevation error per zone, the recall of landable cells, and the '
     'number of *dangerous* false accepts, cells declared landable whose '
     'true slope or step exceeds twice its limit. Results are reported as '
     'medians with interquartile ranges. Because a simulated mission is '
     'not repeatable to the seed, configurations are compared with the '
     'unpaired Mann-Whitney U test (Mann & Whitney, 1947) with the '
     'rank-biserial correlation *r* as effect size, and trends in α with '
     'Spearman\'s rank correlation.'),

    ('h2', '6.2 Perception Against Ground Truth'),
    ('p',
     'The perception results are measured on the reference map of W1, a '
     'survey map that every figure of this section uses and that was never '
     'regenerated ({fig:w1}). {fig:mapvstruth} compares the map with the '
     'truth. On flat ground the elevation error is 0.240 m RMS with a bias '
     'of 0.014 m after removal of the constant offset of the estimator '
     'origin; the 3° ramp is mapped to 0.128 m RMS. Two zones stand out '
     'for opposite reasons. The 10° ramp has a larger error (0.466 m RMS), '
     'concentrated on its raised edges. The cube is mapped 1.67 m too low '
     'on average (2.11 m RMS): a single Gaussian per cell cannot represent '
     'both the top of a structure and the ground beside it, so the map '
     'shrinks raised structures by about 1.5 m at their edges. This '
     'limitation is inherent to the representation and is not corrected '
     'here; its consequences are bounded by the obstacle inflation and, as '
     'shown below, are conservative.'),
    ('fig', 'w1', FIGS + 'f3_validation_world.png', 16.0,
     'The validation world W1. Left: rendered by Gazebo Harmonic from the '
     'south-south-west. Right: the truth zones from above, with the '
     'geometric test each zone represents.'),
    ('fig', 'mapvstruth', FIGS + 'f4_map_vs_truth.png', 16.0,
     'Reference map of W1 against the truth: map height, true height, and '
     'their difference (clipped at ±0.5 m), with the bias, RMS error and '
     'coverage per zone. The cube is read 1.67 m too low on average '
     'because one height per cell cannot represent both the top and the '
     'ground beside it.'),
    ('p',
     '{fig:slope} justifies the slope threshold. On truly flat ground the '
     'slope read on the map has a 95th percentile of 4.34°; on the 10° '
     'ramp the 1st percentile is 8.25°; a threshold of 6° therefore '
     'separates the two for 95 % and 99 % of their cells respectively. '
     'The threshold is also not too strict in the other direction: about '
     '1 % of the cells of the 3° ramp read above 6° (99th percentile '
     '6.17°), a small and conservative loss. The tail of the flat '
     'distribution (99th percentile 21.66°) is not noise. Of the 88 flat '
     'cells (3.5 %) that read above 6°, 81 lie within 3 m of the edge of '
     'another zone, where the truth has a real step of up to 2.73 m (the '
     'side of the 10° ramp) or 4 m (the cube), while the truth under all '
     '88 is flat: the representation smears these steps beyond the '
     'one-cell margin of the analysis window. {fig:sloperough} verifies '
     'the separation of descriptors: the zone medians fall where they '
     'should, slope without roughness on the ramps (2.9° and 10.4° with '
     'roughness 0.013 and 0.014 m) and roughness without excessive slope '
     'on the rough patch (0.085 m against 0.150 m in the truth, 4.4°).'),
    ('fig', 'slope', FIGS + 'f5_slope_noise.png', 15.0,
     'Distribution of the slope read on the map per zone of W1, with the '
     '6° threshold. The threshold lies between the 95th percentile of flat '
     'ground (4.34°) and the 1st percentile of the 10° ramp (8.25°).'),
    ('fig', 'sloperough', FIGS + 'f6_slope_roughness.png', 12.5,
     'Slope against roughness read on the map, one point per cell, by '
     'zone (log scales), with the two thresholds and the landable '
     'quadrant shaded; the large markers are the zone medians. Ramps read '
     'as slope and the rough patch as roughness: the descriptors are '
     'separated.'),
    ('p',
     '{fig:zones} brings the chain together. With the 1.5 m footprint the '
     'detector recovers 94.2 % of the truly landable cells. It produces '
     '46 false accepts, all benign (cells slightly beyond a limit, such '
     'as a gentle slope just over 6°), and no dangerous one. Two '
     'qualifications are essential. First, the zero is a joint property of '
     'the map and the detector: on earlier maps the corrected footprint '
     'element reduced the dangerous false accepts without eliminating '
     'them. Second, a single map is an observation, not a result. Over '
     'the 80 final maps of an earlier W1 campaign, 14 carried a dangerous '
     'cell, always on the same two cells of the rough patch, where the map '
     'reads a 0.12 to 0.19 m step for a true 0.52 m; a guard added after '
     'that finding confirms a zone only if it contains a whole footprint '
     'of landable cells, and no mission of the final campaigns landed on a '
     'cell that is dangerous in the truth.'),
    ('fig', 'zones', FIGS + 'f7_zones_vs_truth.png', 14.5,
     'Landing cells detected on the reference map against those landable '
     'in the truth (footprint 1.5 m). Recall 94.2 %; 46 false accepts, all '
     'benign; no dangerous false accept.'),

    ('h2', '6.3 Energy Prediction and Landing Precision'),
    ('p',
     '{fig:energy} shows the calibration of Section 4.5. Predicted and '
     'measured energies lie on the identity line for hover segments, '
     'transits and the four validation flights. The right panel is the '
     'reason for the leg term: with two terms the transit error is '
     'structured by leg length, every leg paying the same two '
     'accelerations; with three it collapses to noise. Predicted from the '
     'plan alone, as C1 uses it, the validation flights fall within 0.13 '
     'to 0.26 % of the measurement, slightly below it; this residual '
     'optimism is two orders of magnitude smaller than the reserve.'),
    ('fig', 'energy', FIGS + 'f8_energy_calibration.png', 16.0,
     'Energy model calibration. Left: predicted against measured energy '
     'for every hover and transit segment of the calibration flights and '
     'for the four validation flights never fitted, with the parameters '
     'and their 95 % intervals. Right: relative error of the transits by '
     'leg length, with two terms and with the adopted three, and of the '
     'validation flights predicted from the plan.'),
    ('p',
     'The footprint depends on how precisely the vehicle lands. Over 15 '
     'landings commanded on the same setpoint, the radial error of the '
     'touchdown was 0.124 m on average, 0.180 m at the 95th percentile '
     'and 0.195 m at most ({fig:landing}); most of it is already present '
     'when the land command is sent, the drift during the descent being '
     '0.055 m on average. With the blade tips 0.386 m from the centre, the '
     '1.5 m footprint leaves room for an error of 0.364 m, twice the '
     'measured 95th percentile.'),
    ('fig', 'landing', FIGS + 'f14_landing_precision.png', 15.0,
     'Landing precision against the footprint. Left: radial error of 15 '
     'landings, with the mean, the 95th percentile, and the error each '
     'footprint can absorb (0.364 m at the 1.5 m in use, 0.214 m at the '
     '1.2 m computed from the precision alone). Right: the same seen from '
     'above, with the airframe drawn at the largest measured error.'),

    ('h2', '6.4 Safety: the Invariant Against the Baselines'),
    ('p',
     '{tab:safety} gives the central result. No mission with C1 violated '
     'the invariant on any world, 0 of 150, and none used its reserve. B0, '
     'which ignores energy, violated it in every mission and ended in an '
     'emergency landing wherever its budget ran out, three times on W3 on '
     'a cell that is dangerous in the truth. Against B0 the difference in '
     'violations is significant on every world (p ≤ 0.0001, *r* = −1.00).'),
    ('tab', 'safety', 'Invariant I1 per world and configuration, ten '
     'missions per cell (B3 pools its five values of α, 50 missions).',
     ['World', 'Configuration', 'Missions violating I1',
      'Emergency landings', 'Energy left at landing (median)'],
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
     'pairing of its threshold with the world and the budget, not on any '
     'property of the policy. On W1 at 16 kJ, 30 % of the budget is less '
     'than the return, the landing and the reserve together, and B1 '
     'violated I1 in every mission, landing 1.08 kJ into its reserve each '
     'time; {fig:chrono} shows one such mission next to a C1 mission of '
     'the same world. On W2 at 48 kJ the same threshold covers the return '
     'from anywhere in the area, and B1 held the invariant in all ten '
     'missions; on W3 it failed once. A fixed threshold can be made safe '
     'for one world by tuning it, but only a guarantee computed from the '
     'actual registry and position is safe without tuning, which is the '
     'property C1 provides.'),
    ('fig', 'chrono', W1 + 'f9_f10_chronogram.png', 15.0,
     'The guarantee over one flight on W1 at 16 kJ: energy left against '
     'the energy needed to return and land plus the reserve. Top: B1, '
     'which returns at 30 % of the budget; I1 is violated before the '
     'return starts (shaded) and the vehicle lands below its reserve. '
     'Bottom: B3 at α = 0.75, whose return is triggered by C1 when no '
     'frontier remains admissible; the margin stays positive throughout.'),
    ('p',
     'The guarantee costs coverage only relative to policies that break '
     'it. On W1, B1 covered 0.89 of the area against 0.69 for B3 at '
     'α = 0.5: the coverage B1 gains is the energy it takes from its '
     'reserve. On W2 and W3, where B1 does not overspend, C1 with C2 '
     'covered more than B1 (0.70 against 0.54 on W2, 0.63 against 0.50 '
     'coverage seen on W3 at α = 0.5; p = 0.0002, *r* = +1.00 on both), '
     'and landed with less energy left (6.4 to 7.0 kJ against 8.7 kJ on '
     'W2, 9.6 to 11.0 kJ against 17.6 kJ on W3): the margin that B1 '
     'wastes on a conservative threshold, C1 spends on exploration. B0 '
     'covers the most on every world, by consuming its reserve and '
     'landing wherever it stops.'),

    ('h2', '6.5 The Coverage and Margin Trade-off'),
    ('p',
     '{tab:alpha} and {fig:metrics} report the effect of α. On all three '
     'worlds the coverage peaks in the interior of the range, at α = 0.75 '
     '(within 0.01 of α = 0.5 on W2 and W3), and α = 0.75 covers '
     'significantly more than both ends: 0.77 against 0.65 (α = 0, '
     'p = 0.0002) and 0.76 (α = 1, p = 0.0091) on W1; 0.69 against 0.61 '
     'and 0.59 on W2 (p = 0.0002 and 0.0006); 0.60 against 0.48 and 0.52 '
     'on W3 (p = 0.0002 and 0.0006). Cells observed per kilojoule follow '
     'the same pattern. This comparison was fixed on W1 before the W2 and '
     'W3 campaigns, so its replication on the two other worlds is an '
     'out-of-sample test rather than a selection.'),
    ('tab', 'alpha', 'Coverage (fraction of the explored area OBSERVED), '
     'median with interquartile range, ten missions each.',
     ['Configuration', 'W1, 16 kJ', 'W2, 48 kJ', 'W3, 80 kJ'],
     [['B0, nearest frontier', '0.87 [0.86, 0.88]', '0.78 [0.73, 0.80]',
       '0.60 [0.56, 0.62]'],
      ['B1, gain, 30 % return', '0.89 [0.89, 0.90]', '0.54 [0.48, 0.56]',
       '0.47 [0.43, 0.49]'],
      ['B3, α = 0', '0.65 [0.64, 0.66]', '0.61 [0.61, 0.62]',
       '0.48 [0.44, 0.49]'],
      ['B3, α = 0.25', '0.67 [0.66, 0.67]', '0.68 [0.67, 0.69]',
       '0.56 [0.53, 0.62]'],
      ['B3, α = 0.5', '0.69 [0.67, 0.71]', '0.70 [0.67, 0.72]',
       '0.60 [0.60, 0.63]'],
      ['B3, α = 0.75', '**0.77 [0.77, 0.78]**', '**0.69 [0.67, 0.70]**',
       '**0.60 [0.58, 0.60]**'],
      ['B3, α = 1', '0.76 [0.67, 0.77]', '0.59 [0.57, 0.61]',
       '0.52 [0.47, 0.56]']],
     [4.6, 3.8, 3.8, 3.8]),
    ('p',
     'The mechanism behind the optimum is visible in the decisions. The '
     'number of frontiers chosen per mission falls monotonically with α '
     '(on W3: 40, 34.5, 27.5, 26 and 21; Spearman ρ = −0.96, p < 0.0001; '
     'on W2: 19, 19, 16, 15, 12), and the mean distance to the nearest '
     'confirmed zone rises with it (ρ = +0.48 on W3, p = 0.0004). At '
     'α = 0 the vehicle chooses the frontier that leaves the most energy, '
     'which keeps it near its zones in many short trips. At α = 1 it goes '
     'for the largest gain first, often far away, and C1 then finds no '
     'second admissible round trip; on W1 under the 20 Oct code, pure gain '
     'returned after two choices with 4.0 kJ unused. The intermediate '
     'weights explore far enough to see new ground while keeping the '
     'second and third trips admissible. On W1 the minimum margin of a '
     'mission also decreases with α (ρ = −0.36, p = 0.01), which is the '
     'trade-off C2 is designed to expose: more weight on information buys '
     'coverage with margin. {fig:traj} shows the resulting trajectories on '
     'W2: B0 covers the field and lands where it stops, B1 makes long '
     'radial excursions, and B3 sweeps around the central pad in loops '
     'whose reach grows with α.'),
    ('fig', 'metrics', W3 + 'f11_metrics.png', 15.5,
     'Mission metrics on W3 (80 kJ), one dot per mission, medians as bars: '
     'coverage, minimum I1 margin, energy left at landing and cells '
     'observed per kilojoule, for the baselines and B3 at each α. The '
     'lone α = 1 mission at 22 % coverage and 55 kJ left was ended early '
     'by the escape behaviour discussed in Section 7.'),
    ('fig', 'traj', W2 + 'f12_trajectories.png', 16.0,
     'Every flight of the W2 campaign by configuration, over the truth '
     '(flat pads in white, 20° field shaded), with the landings and the '
     'first violation of I1 of each mission. Only B0 violates I1 and lands '
     'off the pads.'),

    ('h2', '6.6 Registry Dynamics and the Loss of a Zone in Use'),
    ('p',
     'The registry is not static in practice. In the 50 C1 missions of W3 '
     'it shrank 90 times without any intervention; 35 of these removed '
     'the zone the vehicle was relying on and 8 the target of a return. '
     'Each was answered at the same cycle and none was followed by a '
     'violation. On W2, whose three pads are large and open, the registry '
     'shrank twice, and on W1 never. The demotions concentrate on W3 '
     '(49 in 70 missions, against 2 on W2 and 1 on W1), and their cause '
     'was reconstructed by replaying the perception code on the map '
     'snapshots around each event. Of the 637 cells newly refused in the '
     'examinable cases, 609 failed the roughness test, their roughness '
     'jumping from 0.013 to 0.066 m (median) while their own variance did '
     'not change; the truth under them is flat (only 4 are truly rough), '
     'and they lie a median 3 m from a building, tree, car or rock. The '
     'most probable mechanism is the single-height representation seen '
     'obliquely: as the vehicle flies past an obstacle, returns from its '
     'side fall into the cells at its foot and lift a neighbour by about '
     '0.2 m, which raises the roughness of the adjacent windows above the '
     'threshold. The refusals are false and therefore conservative for '
     'safety; their cost is the demotions and the returns they divert. '
     'The oblique source of the points is inferred rather than observed, '
     'since point clouds are not logged.'),
    ('p',
     'To test the reaction of Section 4.4 deliberately, a field of 36 '
     'crates (2 m cubes on a 3.5 m pitch, covering a 20 m square, with gaps '
     'too narrow for a footprint) was placed by script over the target of '
     'a return while the vehicle was 35 to 20 m from it, on W3 at α = 1, '
     'the configuration that returns to small and distant zones most '
     'often. The 20 m lower bound keeps the event before the commitment '
     'to land, after which nothing is re-evaluated by design. An earlier '
     'scenario placing the obstacle at a frontier transit relying on a '
     'designated pad never fired in 26 missions: a frontier transit '
     'rarely relies on a single zone and flies away from it, whereas a '
     'return always flies towards the zone it relies on and does so with '
     'the least energy left, which makes it both the common and the '
     'hardest case.'),
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
     'the crates were seen and the zone left the CONFIRMED state 3.1 to '
     '4.1 s after placement, the assumed 2.5 s of detection plus at most '
     'one map cycle; I1 was re-evaluated at that very cycle against the '
     'reduced registry; the return turned to the nearest reachable zone '
     'within 0.5 s; and every landing was a committed landing on another '
     'registry zone, outside the field, on ground that is safe in the '
     'truth. The sequence specified in Section 4.4 is therefore executed as '
     'designed. The result also delimits the guarantee. In three of the '
     'four missions no zone remained within the guarantee after the loss: '
     'C1 returns precisely when no frontier is admissible any longer, so '
     'the margin it keeps at that point is smaller than the detour to the '
     'next zone. The violation was logged, and the vehicle still landed '
     'on the nearest zone using at most 0.30 kJ of its 8 kJ reserve. I1, '
     'as defined, holds as long as the registry does not shrink under the '
     'zone it relies on; when it does, the reserve absorbs the shortfall. '
     'Over all 188 C1 missions flown since the invariant was introduced, '
     'the reserve was touched twice (1.63 kJ before the final corrections '
     'and 0.30 kJ in this provoked case) and the lowest energy at landing '
     'was 2.8 kJ.'),
    ('fig', 's95', S95 + 'f13_B3a100_r13.png', 15.5,
     'Provoked loss of the zone in use during a return on W3 (mission '
     'r13, the smallest zone and the hardest branch). Top: the map before '
     'and after the crate field is seen, with the flight, the reaction '
     'and the landing commit on zone 3. Bottom: energy left against the '
     'energy needed to return and land plus the reserve; I1 is violated '
     'from the loss until the landing commit (shaded), the reserve '
     'covering 0.30 kJ.'),

    ('h2', '6.7 Negative Results and Discussion'),
    ('p',
     'Several results were negative, and each changed the design. The '
     'distance-proportional energy model of the proposal was refuted by a '
     'factor of two hundred on a hover, and the simulator\'s battery '
     'telemetry turned out to measure time rather than charge '
     '(Section 4.5). A measurement variance growing with range, '
     'σ = 0.02 + 0.009 *d*, degraded the flat-ground error from 0.121 to '
     '0.143 m in an isolated test: the dominant error is an attitude error '
     'correlated across the points of a scan, which the independent-'
     'observation assumption of the filter cannot model, so angular '
     'diversity between passes matters more than per-point weighting. A '
     'Mahalanobis gate intended to handle discontinuities raised the '
     'dangerous false accepts from 26 to 138 and lowered the recall from '
     '92.6 to 65.3 % on the map where it was tested, because it locked '
     'each cell to the first surface seen; any fix to P2 must keep two '
     'surface hypotheses per cell. C1 with a constant decision latency '
     'violated I1 seven times on the first large flight (Section 4.6). The '
     'invariant gate itself first failed as written on W3: two of 25 C1 '
     'missions violated I1, both after the loss of their return target; '
     'three corrections, each measured on its own (I1 at arrival charging '
     'the landing only, the robust return switch, and the timing fix of '
     'Section 5.2), brought the count to 0 of 50 on W3 and 0 of 150 '
     'overall. Finally, a well-tuned fixed threshold holds I1 (B1 on W2): '
     'the advantage of C1 is not to beat every threshold but to hold '
     'without tuning per world and budget.'),
    ('p',
     'Taken together, the results answer the sub-questions behind the '
     'research question. The map supports a classification with a recall '
     'near 94 % and no dangerous false accept on the reference map, as a '
     'joint property of map and detector. The calibrated model predicts '
     'energy within 0.3 % on unseen trajectories, far inside the 10 % '
     'acceptance bar. C1 prevents the vehicle from entering a state in '
     'which no confirmed zone is reachable, as long as the registry does '
     'not shrink under the zone in use, at no coverage cost relative to a '
     'baseline that respects the same reserve. And C2 exposes a monotone '
     'trade-off with an interior optimum near α = 0.75 that replicates on '
     'three worlds.'),

    # ------------------------------------------------------------ 7
    ('h1', '7 Conclusion and Future Work'),
    ('h2', '7.1 Conclusion'),
    ('p',
     'This report addressed how a UAV can explore unknown terrain for safe '
     'landing zones while keeping a confirmed zone continuously reachable. '
     'The answer proposed is an energy-feasibility invariant over a '
     'registry of zones discovered online, enforced as a filter on every '
     'exploration decision and monitored at every map update, together '
     'with a utility that trades information against energy margin. '
     'Built on a conventional LiDAR mapping and terrain analysis chain and '
     'an energy model calibrated against rotor power, the system kept the '
     'invariant in all 150 missions in which it was active, across three '
     'worlds, where a coverage-only explorer broke it every time and a '
     'fixed-threshold return broke it whenever its threshold did not suit '
     'the world. The trade-off parameter has an interior optimum that '
     'replicates out of sample. The deliberate removal of a zone in use '
     'showed both the reaction working as designed and the precise scope '
     'of the guarantee: it rests on the zone the vehicle relies on, and '
     'the reserve covers the case where that zone disappears late in a '
     'mission.'),
    ('p',
     'The limitations must be stated with the same precision. All results '
     'are in simulation, without wind, with a static environment, GPS '
     'localisation and a single sensor. The energy is the mechanical rotor '
     'energy of the simulator, which excludes motor and battery losses, '
     'and the model is valid only for the calibrated flight profile. The '
     'take-off point is assumed free for the whole mission, and nothing '
     'is re-evaluated after the landing is committed; the specification '
     'foresaw an exception when an obstacle appears on the landing site, '
     'which is not implemented. The single-height map shrinks raised '
     'structures and produces false refusals near obstacles. And in two of '
     '70 missions on W3 the escape behaviour, which lets the vehicle leave '
     'a region enclosed by inflated obstacles, ended the mission early with '
     '21 and 48 kJ unused, because the machine returns and lands rather '
     'than resuming exploration once out.'),
    ('h2', '7.2 Short-Term Future Work'),
    ('p',
     'With three more months, the first improvements would address these '
     'limits directly. (i) Resume exploration after an escape instead of '
     'landing, which would recover the coverage lost in those missions. '
     '(ii) Implement the landing abort when an obstacle is detected on the '
     'committed site, and remove the assumption that the take-off point '
     'stays free by letting observations retire it like any other zone. '
     '(iii) Replace the single Gaussian per cell by two surface hypotheses, '
     'or use a robust roughness such as the median of absolute residuals, '
     'to suppress the oblique false refusals that drive the demotions on '
     'W3. (iv) Integrate the energy on simulation time rather than system '
     'time, so that a slowed simulation is not charged extra energy. '
     '(v) Offer, as an option, a stricter guarantee that holds without the '
     'zone in use, at the price of a larger margin, and measure its '
     'coverage cost; the quantity it requires is already logged. (vi) Make '
     'missions repeatable to the seed so that configurations can be '
     'compared with paired tests.'),
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
    ('placeholder', '[To be completed by the author, following the '
     'instructions on the Canvas assignment page.]'),

    # --------------------------------------------------- references
    ('h1', 'References'),
    ('ref', 'Abeywickrama, H. V., Jayawickrama, B. A., He, Y., & '
     'Dutkiewicz, E. (2018). Comprehensive energy consumption model for '
     'unmanned aerial vehicles, based on empirical studies of battery '
     'performance. *IEEE Access, 6*, 58383–58394. '
     'https://doi.org/10.1109/ACCESS.2018.2875040'),
    ('ref', 'Ames, A. D., Coogan, S., Egerstedt, M., Notomista, G., '
     'Sreenath, K., & Tabuada, P. (2019). Control barrier functions: Theory '
     'and applications. *2019 18th European Control Conference (ECC)*, '
     '3420–3431. https://doi.org/10.23919/ECC.2019.8796030'),
    ('ref', 'Beycimen, S., Ignatyev, D., & Zolotas, A. (2026). Real-time '
     'terrain traversability analysis and mapping for autonomous robotics '
     'in dynamic environments: Fusing appearance- and geometry-based '
     'approaches. *Journal of Field Robotics, 43*(3), 1719–1738. '
     'https://doi.org/10.1002/rob.70097'),
    ('ref', 'Bircher, A., Kamel, M., Alexis, K., Oleynikova, H., & '
     'Siegwart, R. (2016). Receding horizon "next-best-view" planner for 3D '
     'exploration. *2016 IEEE International Conference on Robotics and '
     'Automation (ICRA)*, 1462–1468. '
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
     'battery. *2015 IEEE International Conference on Robotics and '
     'Automation (ICRA)*, 2230–2235. '
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
     'Gazebo, an open-source multi-robot simulator. *2004 IEEE/RSJ '
     'International Conference on Intelligent Robots and Systems (IROS)*, '
     '2149–2154. https://doi.org/10.1109/IROS.2004.1389727'),
    ('ref', 'Liu, Z., Sengupta, R., & Kurzhanskiy, A. (2017). A power '
     'consumption model for multi-rotor small unmanned aircraft systems. '
     '*2017 International Conference on Unmanned Aircraft Systems '
     '(ICUAS)*, 310–315. https://doi.org/10.1109/ICUAS.2017.7991310'),
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
     'embedded platforms. *2015 IEEE International Conference on Robotics '
     'and Automation (ICRA)*, 6235–6240. '
     'https://doi.org/10.1109/ICRA.2015.7140074'),
    ('ref', 'Papachristos, C., Khattak, S., & Alexis, K. (2017). '
     'Uncertainty-aware receding horizon exploration and mapping using '
     'aerial robots. *2017 IEEE International Conference on Robotics and '
     'Automation (ICRA)*, 4568–4575. '
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
     'exploration. *Proceedings of the 1997 IEEE International Symposium '
     'on Computational Intelligence in Robotics and Automation (CIRA\'97)*, '
     '146–151. https://doi.org/10.1109/CIRA.1997.613851'),

    # ----------------------------------------------------- appendix
    ('pagebreak',),
    ('h1', 'Appendix A: Communication Log with Supervisor'),
    ('placeholder', '[Semester 2 communication log, to be added by the '
     'author using the Canvas template.]'),
]
