"""
Report text, first half: title page, abstract, Sections 1 to 4.

Every number in this file comes from the experiment directories named
next to it in docs/prompt-maitre-projet-uav.md (sections 4, 7 and 10)
or from the figure index experiments/2026-10-24_thesis_figures/README.md.
"""

FIGS = 'experiments/2026-10-24_thesis_figures/'
NEW = 'experiments/2026-10-31_report_analyses/'

BLOCKS = [
    ('title', {
        'title': 'Energy-Feasible UAV Exploration over an Online Registry '
                 'of Safe Landing Zones',
        'subtitle': 'Capstone B Final Report',
        'author': 'Dorian Veloso',
        'id': 'Student ID 25773615',
        'supervisor': 'Supervisor: Dr Graeme Best',
        'school': 'University of Technology Sydney',
        'date': 'October 2026',
    }),
    ('abstract',
     'A battery-powered rotorcraft exploring unknown terrain must be able '
     'to land safely before its energy runs out, yet the places where it '
     'could land are not known in advance: they have to be discovered '
     'during the flight, and a later, closer observation may withdraw '
     'them. This report presents an exploration planner that detects '
     'geometrically safe landing zones online from a downward 3D LiDAR, '
     'holds them in a dynamic registry, and enforces an energy-feasibility '
     'invariant: whenever a confirmed zone exists, the energy left covers '
     'the flight to one of them and the landing, plus a fixed reserve. Two '
     'mechanisms are proposed. C1 filters every candidate frontier against '
     'this invariant and re-evaluates it whenever the registry changes, '
     'including when it shrinks. C2 ranks the admissible frontiers by a '
     'convex combination of information gain and energy margin, weighted '
     'by a parameter α. Both rest on an energy model calibrated on rotor '
     'shaft power in simulation, E = P_{hover} t + k_{move} d + '
     'c_{leg} n_{legs}, whose transit predictions fall within 0.16 % of '
     'the measurement. In 210 PX4 software-in-the-loop missions over three '
     'worlds, no mission with C1 violated the invariant (0/150), against '
     '30/30 for nearest-frontier exploration and 11/30 for a classical '
     'explorer returning at a fixed 30 % battery threshold; in flight, the '
     'energy C1 charged exceeded the energy measured in 2530 of 2535 '
     'transits. For coverage the energy-margin utility has an interior '
     'optimum, α between 0.5 and 0.75, significantly above both pure margin '
     'and pure gain on all three worlds, whereas the confirmed landing area '
     'found keeps rising with α. A deliberately provoked loss of the zone in '
     'use during a return was detected within 3.1 to 4.1 s and answered at '
     'the same decision cycle; it also exposes the limit of the guarantee, '
     'which holds as long as the registry does not shrink under the zone '
     'it relies on.'),
    ('keywords', 'UAV exploration, safe landing zone detection, energy '
     'constraints, reachability, frontier-based exploration, LiDAR '
     'terrain mapping'),
    ('pagebreak',),
    ('count_on',),

    # ------------------------------------------------------------ 1
    ('h1', '1 Introduction'),
    ('p',
     'Rotary-wing UAVs are increasingly sent where no prepared landing pad '
     'exists: post-disaster assessment, coastal and environmental '
     'monitoring, inspection of remote infrastructure. In these missions '
     'the terrain is unknown at launch and endurance is measured in tens of '
     'minutes. The vision behind this work is a UAV that can be released '
     'over unknown terrain, explore it autonomously, and at every moment '
     'still know where it can put itself down safely with the energy it '
     'has left. Manned aviation offers a structural analogue: a pilot must '
     'always keep a reachable alternate aerodrome given fuel, weather and '
     'runway constraints. For a UAV over unknown terrain the alternate '
     'must instead be discovered in flight, and re-verified as the battery '
     'drains and as the map, and the alternate with it, is revised.'),
    ('p',
     'The capabilities this requires have so far been developed separately. '
     'Frontier-based and next-best-view exploration planners maximise '
     'coverage or information gain without reasoning about where the '
     'vehicle could land (Yamauchi, 1997; Bircher et al., 2016). Landing '
     'site detectors assess slope, roughness and obstacles on a given map, '
     'but do not steer the exploration that produces the map (Shah Alam & '
     'Oluoch, 2021; Takahashi et al., 2013). Energy-aware planners either '
     'trigger a return to a predefined base when the battery crosses a '
     'threshold (Gramajo & Shankar, 2017; Gasche et al., 2025), or reduce '
     'consumption without checking that any safe state remains reachable '
     '(Elskamp et al., 2026); work that does provide formal reachability '
     'guarantees assumes that the safe sets are known in advance or found '
     'incidentally (Cherenson et al., 2025). None addresses the case in '
     'which safe landing sites are initially absent, are discovered online '
     'by terrain assessment, and may later be withdrawn when a closer '
     'observation contradicts the first one. This leads to the research '
     'question of this work:'),
    ('quote',
     'How can a UAV autonomously search an unknown environment for safe '
     'landing zones, while ensuring continuous reachability of a '
     'geometrically confirmed safe site whenever one has been identified?'),
    ('p',
     'The final clause is not a stylistic precaution. At take-off no safe '
     'site is known, so an unconditional guarantee could not be stated and '
     'the mission could never begin; the problem must therefore define what '
     'is guaranteed before the first confirmation and how the guarantee '
     'behaves when the set of confirmed sites changes (Section 3).'),
    ('p',
     'We address the question with a planner that adds two mechanisms to '
     'a conventional perception and exploration pipeline ({fig:overview}). '
     'LiDAR returns are fused into a 2.5D elevation map, tested for slope, '
     'roughness and step height, and grouped into landing zones held in a '
     'dynamic registry, where a zone becomes usable only once a whole '
     'landing footprint has been seen on two passes. The first '
     'contribution, C1, is an energy-feasibility invariant over this '
     'registry: a frontier is admissible only if the energy to reach it, '
     'return from it to the nearest confirmed zone and land fits within '
     'the energy left minus a reserve, and the invariant is re-checked at '
     'every map update so that the loss of a zone is answered at once. '
     'The second, C2, ranks the admissible frontiers by U = αG + (1 − α)M, '
     'a convex combination of information gain and energy margin rescaled '
     'at each decision. Both rest on an energy model calibrated against '
     'rotor shaft power.'),
    ('p',
     'The system was evaluated in PX4 software-in-the-loop simulation on '
     'three worlds, 210 missions in all, against nearest-frontier '
     'exploration (B0) and a gain-driven explorer returning at a fixed 30 % '
     'battery threshold (B1), the common practice. No mission with C1 '
     'violated the invariant (0 of 150), against 30 of 30 for B0 and 11 of '
     '30 for B1, whose outcome depends on whether its threshold suits the '
     'world. For coverage, C2 has an interior optimum between α = 0.5 and '
     '0.75 on all three worlds; for the landing area found, more weight on '
     'information is better. A deliberately provoked loss of the zone in '
     'use was answered at the cycle it was detected and exposed the limit '
     'of the guarantee, which the reserve then covered.'),
    ('p',
     'The contributions are: (i) an energy-feasibility invariant over a '
     'registry of landing zones that is built online and can shrink, with '
     'an explicit treatment of the bootstrap case and of zone loss; (ii) '
     'an energy-margin utility whose per-decision normalisation makes its '
     'parameter act over the whole range; and (iii) an empirical '
     'characterisation: an energy model checked against every leg flown, '
     'a 210-mission evaluation with criteria fixed in advance, an account '
     'of what the filter and the utility do at each decision, a provoked '
     'registry shrink, and documented negative results.'),

    # ------------------------------------------------------------ 2
    ('h1', '2 Related Work'),
    ('p',
     'This work draws on four bodies of literature: autonomous '
     'exploration, landing site detection, terrain mapping, and '
     'energy-constrained planning. Each is mature on its own; the gap lies '
     'in how they are coupled.'),
    ('h2', '2.1 Autonomous Exploration of Unknown Environments'),
    ('p',
     'Frontier-based exploration (Yamauchi, 1997) reduces exploration to '
     'repeatedly travelling to the boundary between known free space and '
     'unknown space. Its appeal lies in its simplicity and in the fact '
     'that its candidates are explicit places on the map. Sampling-based '
     'next-best-view planners extend the idea to three-dimensional '
     'volumetric maps (Bircher et al., 2016), usually stored as octrees '
     '(Hornung et al., 2013), with an information gain estimated by ray '
     'casting from each candidate view. Later work made the exploration '
     'objective aware of localisation uncertainty (Papachristos et al., '
     '2017) and of risk under stochastic dynamics (Han, 2023), part of a '
     'broader shift, articulated in the SLAM literature (Cadena et al., '
     '2016), from map building as an end towards perception in the '
     'service of a task.'),
    ('p',
     'The common thread is that the exploration objective remains '
     'information. Energy enters, when it does, as a path-length cost, and '
     'the availability of a place to land, which is what actually bounds a '
     'battery-powered rotorcraft, plays no role in the decision. This work '
     'keeps the frontier paradigm because its candidates are discrete and '
     'explicit: each can be checked against an energy constraint before it '
     'is ranked, the structure C1 and C2 require.'),
    ('h2', '2.2 Safe Landing Site Detection'),
    ('p',
     'Shah Alam and Oluoch (2021) classify landing zone detection into '
     'geometry-based, vision-based and hybrid learning-based methods, and '
     'note that geometric methods offer the most reliable metric '
     'assessment outdoors. They also have the longest history in '
     'safety-critical settings: LiDAR hazard detection for planetary '
     'landing fitted local planes to an elevation map and thresholded '
     'their slope and the residual roughness (Johnson et al., 2002), the '
     'same descriptors used in this work. On rotorcraft, Scherer et al. '
     '(2012) landed a full-scale helicopter at unprepared sites assessed '
     'from LiDAR during the approach, and Takahashi et al. (2013) '
     'benchmarked safe landing area determination against surveyed field '
     'sites. Visual SLAM such as ORB-SLAM3 (Campos et al., 2021) produces '
     'dense maps from cameras, but the height accuracy needed for slope '
     'and step tests still favours ranging sensors.'),
    ('p',
     'Two conclusions follow. Geometric tests on a 2.5D map are mature '
     'enough to be a building block rather than a research object. And '
     'the literature treats landing detection as a perception problem: '
     'sites are assessed on a map built for another purpose, during a '
     'final approach, or offline, so the result neither steers the '
     'exploration nor is checked against the energy needed to reach the '
     'site. Detectors are also usually evaluated on a single map, whereas '
     'Section 6.2 shows that their dangerous false accepts must be '
     'measured over many.'),
    ('h2', '2.3 Terrain Mapping and Traversability'),
    ('p',
     'Elevation mapping with per-cell uncertainty is the standard '
     'representation for this purpose: each cell holds a height estimate '
     'and its variance, updated by a scalar Kalman filter (Kalman, 1960) '
     'as new returns arrive, with pose uncertainty propagated into the map '
     '(Fankhauser et al., 2014, 2018). Traversability analysis then '
     'derives local descriptors from the map. Beycimen et al. (2026) show '
     'that slope, roughness and step height discriminate safe from '
     'hazardous terrain in real time in the field, and Brubaker et al. '
     '(2013) that roughness can be recovered from LiDAR returns. These '
     'techniques are reused here unchanged; their error characteristics '
     'set the thresholds, the footprint and hence the safety margin. '
     'One known limit, a single height per cell that cannot represent two '
     'surfaces, turns out to drive both the false refusals near obstacles '
     '(Section 6.6) and the residual dangerous accepts (Section 6.2).'),
    ('h2', '2.4 Energy-Constrained Planning and Reachability'),
    ('p',
     'Empirical studies of multirotor consumption show that power is '
     'dominated by the cost of staying airborne rather than by the '
     'distance flown (Liu et al., 2017; Abeywickrama et al., 2018), which '
     'this work confirms (Section 4.5). Planning under a '
     'battery budget has mostly taken the form of a return-to-base '
     'problem. Gramajo and Shankar (2017) redirect a coverage mission '
     'towards a known exit state when the remaining energy falls below a '
     'threshold; Cesare et al. (2015) coordinate multi-UAV exploration '
     'with vehicles returning to a base under battery and communication '
     'limits; Gasche et al. (2025) trigger a model-predictive return to '
     'base on the depth of discharge. Elskamp et al. (2026) integrate '
     'trajectory-level energy prediction into exploration and reduce total '
     'consumption, but do not verify that a safe state remains reachable.'),
    ('p',
     'Formal guarantees exist in a different line of work. Cherenson et '
     'al. (2025) maintain backup trajectories to renewal sets under budget '
     'constraints, so that a safe state stays reachable, but the renewal '
     'sets are known a priori or found incidentally. Control '
     'barrier functions express safety as the forward invariance of a '
     'safe set (Ames et al., 2019); C1 can be read as a discrete-time, '
     'set-valued analogue in which the safe set is defined by an energy '
     'inequality over a registry that itself changes with perception. In '
     'all of these works the place to return to is fixed, given or '
     'incidental, and none considers that it may disappear.'),
    ('h2', '2.5 Synthesis and Research Gap'),
    ('p',
     '{tab:positioning} summarises the positioning. Three gaps emerge. '
     'First, the availability of landing sites does not influence where '
     'an exploring vehicle goes next. Second, energy guarantees, when they '
     'exist, are stated relative to a static and predefined return target. '
     'Third, no work treats the case where the safe set shrinks because '
     'perception revises an earlier assessment while the vehicle relies on '
     'it. This work addresses the three gaps for a single vehicle in '
     'simulation. It deliberately contributes nothing to mapping, terrain '
     'analysis, frontier detection or path planning, which are taken from '
     'the works above; the contribution is the coupling between a registry '
     'of safe zones discovered online and a continuous energy guarantee, '
     'and the selection policy that follows from it.'),
    ('tab', 'positioning',
     'Positioning of this work against representative prior work.',
     ['Work', 'Exploration objective', 'Landing assessment',
      'Return target', 'Energy guarantee'],
     [['Yamauchi (1997); Bircher et al. (2016)', 'coverage, information',
       'none', 'none', 'none'],
      ['Johnson et al. (2002); Scherer et al. (2012); Takahashi et al. '
       '(2013)', 'none (given map or final approach)', 'geometric, LiDAR',
       'not applicable', 'none'],
      ['Gramajo & Shankar (2017); Cesare et al. (2015); Gasche et al. '
       '(2025)', 'coverage', 'none', 'predefined base',
       'threshold or predictive return trigger'],
      ['Elskamp et al. (2026)', 'information per unit energy', 'none',
       'not verified', 'none (consumption reduced)'],
      ['Cherenson et al. (2025)', 'budget-constrained tasks', 'none',
       'renewal sets, known or incidental', 'formal, backup trajectories'],
      ['This work', 'coverage, traded against margin (C2)',
       'geometric, online registry', 'nearest confirmed zone; may shrink',
       'invariant checked at every decision (C1)']],
     [3.6, 3.0, 2.8, 3.1, 3.5]),

    # ------------------------------------------------------------ 3
    ('h1', '3 Problem Statement'),
    ('h2', '3.1 Setting and Assumptions'),
    ('p',
     'A multirotor UAV takes off from a known point into terrain that is '
     'unknown, static and contained in a square exploration area Ω. It '
     'carries a single downward-facing 3D LiDAR and is localised by GPS '
     'through the autopilot estimator; no SLAM is used, following the '
     'supervisor\'s advice in the first semester. It flies at a fixed '
     'cruise altitude *h* above the take-off point (8 m in all experiments) '
     'and has an energy budget *B*_{0}. The energy left at time *t* is '
     '*B*_{t}, and a fixed reserve *R* = 0.1 *B*_{0} is withheld from '
     'planning. There is no wind. The energy budget is a logical budget '
     'measured by integrating rotor shaft power in simulation '
     '(Section 4.5), not the autopilot\'s simulated battery. '
     '{tab:notation} collects the notation used from here on.'),
    ('tab', 'notation', 'Notation. Values are those of every experiment '
     'unless a world is named.',
     ['Symbol', 'Meaning', 'Value or definition'],
     [['Ω; *h*', 'exploration area (square); cruise altitude above take-off',
       '60, 120, 160 m side (W1, W2, W3); 8 m'],
      ['*B*_{0}; *B*_{t}; *R*', 'energy budget; energy left at time *t*; '
       'reserve withheld from planning', '16, 48, 80 kJ; measured; '
       '0.1 *B*_{0}'],
      ['*H*_{t}', 'elevation map: height, variance and count per cell',
       '1 m cells'],
      ['*Z*_{t}', 'registry: the CONFIRMED zones, provisional take-off zone '
       'included', 'Section 4.4'],
      ['*F*_{t}; *f*; *A*', 'frontiers; a frontier viewpoint; frontiers '
       'admissible under C1', 'Section 3.2; {eq:c1}'],
      ['*E*_{go}; *E*_{ret}; *E*_{land}; *E*_{tot}', 'predicted energy to '
       'reach *f*; to return from a point to a zone; to land; their sum',
       '{eq:energy}; *E*_{land} = 3387 J'],
      ['*m*_{t}', 'margin of the vehicle against Invariant 1 (I1)',
       '{eq:margin}'],
      ['*G*; *M*; *U*; α', 'information gain; share of the usable budget '
       'left; utility of C2; weight of gain in C2', '{eq:share}, '
       '{eq:utility}; α ∈ [0, 1]'],
      ['*P*_{hover}; *k*_{move}; *c*_{leg}; *n*_{legs}', 'hover power; '
       'cost per metre; cost per straight leg; legs of a path',
       '{tab:energy}'],
      ['τ', 'decision latency charged as hover', 'measured, Section 4.6'],
      ['*r*_{a}; *r*_{f}', 'analysis and footprint radius', '1 cell each'],
      ['*r*_{in}; *r*_{out}', 'radii that open and close a pass over a '
       'cell', '14.4 m; 18 m'],
      ['I1 to I5', 'the five invariants of the system', 'Section 3.3'],
      ['B0; B1; B3', 'configurations flown', 'Section 6.1']],
     [3.4, 7.4, 5.2]),
    ('h2', '3.2 Map, Landable Cells and Registry'),
    ('p',
     'The terrain is represented by a 2.5D grid *H*_{t} with 1 m cells, '
     'each holding a height estimate, its variance and an observation '
     'count. A cell is *landable* when it is observed, its slope, '
     'roughness and step height are below fixed thresholds, and every cell '
     'under a landing footprint centred on it passes the same tests '
     '(Section 4.3). Connected landable cells form zones. The registry '
     '*Z*_{t} is the set of zones whose state is CONFIRMED, that is, zones '
     'whose footprint cells have each been observed on at least two '
     'distinct passes (invariant I4, Section 4.4). *Z*_{t} grows when a '
     'zone is confirmed and shrinks when new observations refuse its '
     'cells. The frontier set *F*_{t} contains the clusters of observed '
     'cells that border unknown cells, each with a reachable viewpoint '
     '*f* and an information gain *G*(*f*).'),
    ('h2', '3.3 The Energy-Feasibility Invariant'),
    ('p',
     'Let *E*_{ret}(*p*, *z*) be the predicted energy to fly from position '
     '*p* to zone *z* along a collision-free path, and *E*_{land} the '
     'energy of a controlled descent and landing. The margin of the '
     'vehicle at position *p*_{t} is'),
    ('eq', 'margin',
     '*m*_{t} = *B*_{t} − *R* − *E*_{land} − min_{z∈Z} *E*_{ret}(*p*_{t}, *z*),'),
    ('p',
     'where *Z* denotes the registry at the time of evaluation. Five '
     'invariants define what "safe" means in this work. The first is the '
     'one the research question asks for; the other four bound what the '
     'perception and the planner may do:'),
    ('ul', [
        '**I1.** Whenever at least one zone is confirmed, the energy left '
        'covers the flight to the nearest confirmed zone and the landing, '
        'with the reserve untouched.',
        '**I2.** No cell is declared landable if a cell of its analysis '
        'window has never been observed.',
        '**I3.** The vehicle is never commanded over a cell whose known '
        'height exceeds the cruise altitude minus a vertical margin.',
        '**I4.** A zone enters the registry only after every cell of a '
        'landing footprint has been observed on two distinct passes.',
        '**I5.** The land command is issued only over a confirmed zone, '
        'except in an energy emergency, which is logged as such.']),
    ('p', 'Formally, with the margin {eq:margin}:'),
    ('eq', 'i1', 'I1:   *Z*_{t} ≠ ∅   ⇒   *m*_{t} ≥ 0   for all *t*.'),
    ('p',
     'A cycle in which *Z*_{t} is non-empty and *m*_{t} < 0 is a '
     '*violation*: the vehicle can no longer reach any confirmed zone and '
     'land with its reserve intact. The decision rule that keeps I1 true '
     'by construction is the admissibility condition on frontiers,'),
    ('eq', 'c1',
     '*E*_{go}(*p*_{t}, *f*) + min_{z∈Z} *E*_{ret}(*f*, *z*) + *E*_{land} '
     '≤ *B*_{t} − *R*,'),
    ('p',
     'which requires that after flying to *f* the vehicle could still '
     'return to a confirmed zone and land. If the predictions are accurate '
     'and the registry does not change during the transit, choosing only '
     'admissible frontiers implies *m* ≥ 0 on arrival. Neither condition '
     'holds exactly in practice: predictions err, decisions take time, and '
     'the registry can shrink while the vehicle relies on it. The method '
     'therefore also monitors {eq:i1} at every map update (Section 4.6) '
     'and treats a shrink explicitly (Section 4.4).'),
    ('p',
     'I2 to I4 hold by construction of the code (Sections 4.2 to 4.4 and '
     '5.3), I3 and I4 with unit tests; I1 and I5 are measured in every '
     'mission (Section 6.4).'),
    ('h2', '3.4 Objective, Bootstrap and Outputs'),
    ('p',
     'The objective is to maximise the fraction of Ω that is observed, '
     'subject to I1 whenever *Z*_{t} is non-empty, and to end every mission '
     'by landing on a confirmed zone, never wherever the vehicle happens '
     'to be. The bootstrap case, *t* = 0 with no confirmed zone, is '
     'resolved by registering the take-off point as a *provisional* zone: '
     'the vehicle has just left it, so it can land there. This makes I1 '
     'meaningful from the first cycle without a special case. It departs '
     'from the proposal, which excluded the launch point; the consequence, '
     'that the take-off point is assumed to stay free, is a limitation '
     '(Section 7). The outputs of a mission are the trajectory, the '
     'registry history and the landing site.'),

    # ------------------------------------------------------------ 4
    ('h1', '4 Method'),
    ('h2', '4.1 Overview and Changes from the Proposal'),
    ('p',
     '{fig:overview} shows the method as concepts. The perception chain '
     '(top row) turns LiDAR scans into an elevation map, terrain '
     'descriptors, landable cells and finally a registry of confirmed '
     'zones. The decision chain (bottom row) extracts frontiers from the '
     'same map, filters them with C1, ranks the survivors with C2, and '
     'hands the choice to a mission state machine that also decides when '
     'to return and land. The energy model feeds both C1 and C2. A single '
     'component holds the decision authority, so C1 and C2 can be disabled '
     'cleanly for the baselines.'),
    ('fig', 'overview', FIGS + 'r1_method_overview.png', 16.0,
     'Method overview: the perception chain (top) and the decision chain '
     '(bottom). Dashed boxes are the contributions of this work, solid '
     'boxes reused techniques.'),
    ('p',
     'Almost every technical choice of the proposal was changed during the '
     'project, each time because a measurement showed that the proposed '
     'choice would not support the guarantee. {tab:changes} lists the '
     'changes and the evidence for each. The research question, the two '
     'contributions and the evaluation logic are unchanged.'),
    ('tab', 'changes',
     'Design changes from the research proposal and the measurement that '
     'motivated each.',
     ['Component', 'Proposal', 'Final design', 'Reason'],
     [['Energy model', '*E* = *k d*, calibrated on PX4 battery telemetry',
       '*E* = *P*_{hover} *t* + *k*_{move} *d* + *c*_{leg} *n*_{legs}, '
       'calibrated on rotor shaft power',
       'SITL telemetry depends only on armed time; *k d* predicts 77 J for '
       'a 60 s hover costing over 17 kJ (Section 4.5)'],
      ['Terrain scoring', 'weighted composite score; slope 10°, roughness '
       '0.3 m, step 0.5 m; 0.5 m grid', 'four separate tests; slope 6°, '
       'roughness 0.05 m, step 0.24 m; 1 m grid', 'thresholds set by the '
       'measured map noise; a weighted sum lets one good descriptor mask '
       'a failing one (Section 4.3)'],
      ['Exploration map', 'ray-cast occupancy grid', 'frontiers on the '
       'observation state of the elevation map', 'one map for exploring '
       'and for judging terrain, so both agree on what is known'],
      ['Return cost', 'bounded Dijkstra on a roadmap', 'A* on the inflated '
       'obstacle grid; one return field per cycle', 'the same planner '
       'serves flight and C1'],
      ['Launch point', 'never assumed safe', 'provisional zone in the '
       'registry', 'gives I1 a meaning from *t* = 0 (Section 3.4)'],
      ['Zone confirmation', 'safe cluster larger than the footprint',
       'whole footprint seen on two passes per cell (I4)', 'single oblique '
       'views confirmed zones that later vanished (Section 4.4)'],
      ['Configurations', '(a) no filter, (b) C1 only, (c) C1 + C2 with α '
       'by 0.2', 'B0, B1 (30 % threshold), B3 with α ∈ {0, 0.25, 0.5, '
       '0.75, 1}', 'B1 is the practice to beat; (b) equals B3 at α = 1 by '
       'construction'],
      ['Statistics', 'means with 95 % confidence intervals', 'medians, '
       'interquartile ranges, Mann-Whitney U, Spearman', 'bounded '
       'distributions with many zeros; runs not seed-repeatable']],
     [2.4, 4.0, 4.6, 5.0]),
    ('h2', '4.2 Elevation Mapping and Observation State'),
    ('p',
     'Each LiDAR return is transformed into the world frame (Section 5.2) '
     'and fused into the cell it falls in by a scalar Kalman filter, '
     '*k* = *p* / (*p* + *r*), with the measurement variance growing with '
     'range as σ = 0.02 + 0.009 *d* metres. The cell variance is floored at '
     '(0.02 m)^{2}: without the floor it decays as 1/*n*, and after a few '
     'hundred observations a cell is frozen and cannot be corrected even '
     'by a much better observation from close range.'),
    ('p',
     'Every cell is assigned an observation state: UNKNOWN when it has no '
     'return, OBSERVED when it has at least 10 returns and a variance below '
     '2 × 10^{-3} m^{2} (the median fused variance at that count), and '
     'SPARSE otherwise. Frontiers and coverage are defined on this state, '
     'and the terrain analysis refuses any window that contains an unknown '
     'cell (invariant I2: nothing is declared safe that has not been '
     'seen).'),
    ('h2', '4.3 Terrain Analysis and Landing Cells'),
    ('p',
     'For each cell a plane is fitted by least squares to the 3 × 3 window '
     'around it. *Slope* is the inclination of that plane, *roughness* the '
     'standard deviation of the residuals to it, and *step* the largest '
     'height difference between the cell and its neighbours. With 5 cm of '
     'noise between neighbours a finite difference produces about 6° of '
     'spurious slope, which the fit reduces roughly threefold. Because the '
     'plane is removed first, roughness is independent of slope by '
     'construction: a smooth ramp reads as slope, a rubble field as '
     'roughness (verified in Section 6.2).'),
    ('p',
     'A cell passes when its slope is at most 6°, its roughness at most '
     '0.05 m and its step at most 0.24 m, and its window is fully '
     'observed. The four tests are applied separately rather than combined '
     'into a weighted score as in the proposal, because a weighted sum lets '
     'a flat but stepped cell compensate its step with its flatness. The '
     'slope threshold sits between the slope read on flat ground and on '
     'the 10° ramp of the reference map (Section 6.2).'),
    ('p',
     'A cell is *landable* only if every cell covered by a landing '
     'footprint of 1.5 m centred on it passes. The footprint element is '
     'an over-approximation of the disc: every cell the disc touches, even '
     'partially, must pass, which at 1 m resolution is the 3 × 3 square. An '
     'earlier element kept only the cells contained in the disc, a single '
     'cell for a 1.5 m footprint, and therefore accepted cells it had '
     'never tested; correcting it reduced the dangerous false accepts on '
     'the reference map from 25 to 0. The landing precision alone '
     '(Section 6.3) would allow 1.13 m, but a 1.2 m footprint, whose '
     'element loses the diagonal cells, reintroduced 2 dangerous false '
     'accepts: the diagonal cells are a margin that the map error '
     'requires, and 1.5 m was kept. With an analysis radius *r*_{a} and a '
     'footprint radius *r*_{f} in cells, an isolated zone must be at least '
     '2(*r*_{a} + *r*_{f}) + 1 cells wide to be detected, 5 m here.'),
    ('h2', '4.4 Dynamic Landing Zone Registry'),
    ('p',
     'Landable cells are grouped into zones by 8-connected flood fill at '
     'every map cycle. Because zones are re-detected from a map that '
     'changes, each new zone is matched to the zones of the previous cycle '
     'by cell overlap, so that a zone keeps its identity and its history. '
     'A zone enters as CANDIDATE and is not usable by C1. It becomes '
     'CONFIRMED when a whole footprint of its landable cells has been '
     'observed on at least two distinct passes, cell by cell (invariant '
     'I4). A pass of a cell begins when the vehicle comes within '
     '*r*_{in} = 14.4 m of it and ends when it moves beyond *r*_{out} = '
     '18 m, the radius of the sensor footprint at cruise altitude; the '
     'hysteresis prevents a vehicle hovering near the boundary from '
     'counting many passes. The rule replaced one that counted passes over '
     'the zone\'s centroid, which offline confirmed zones of which only '
     '538 m^{2} out of 2465 m^{2} had been seen twice. A zone is also '
     'confirmed only if it holds a whole footprint of landable cells '
     '(Section 6.2), and it is demoted or retired when an update refuses '
     'enough of its cells that no whole footprint remains.'),
    ('p',
     'The registry is what makes the safe set dynamic, and its shrinking '
     'is the case no prior work on energy-constrained exploration treats. '
     'When an update removes a confirmed zone, the system (1) detects it '
     'at the registry update; (2) re-evaluates I1 at once; (3) continues '
     'if I1 holds through another zone; (4) otherwise interrupts the '
     'navigation and returns to the nearest reachable zone; and (5) if no '
     'zone is reachable within the guarantee, logs a violation and still '
     'flies to the nearest one, the reserve covering the shortfall. '
     'Section 6.6 provokes this sequence.'),
    ('h2', '4.5 Energy Model'),
    ('p',
     'The proposal planned a model *E* = *k d*, calibrated from the '
     'battery telemetry of the PX4 simulator. Both parts were refuted. The '
     'simulated battery reports a remaining fraction that decreases '
     'linearly with the time the vehicle has been armed, whatever it does. '
     'The signal used instead is the mechanical shaft power of the four '
     'rotors, computed from the motor speed commands that PX4 sends to the '
     'simulator, and validated in hover within 0.2 % of the analytic '
     'prediction from weight and thrust coefficients. Fitted to a survey '
     'flight, the distance model predicts 77 J for a 60 s hover that '
     'costs over 17 kJ: flight time explains 87 % of the survey energy.'),
    ('p', 'The adopted model charges time, distance and each straight leg '
     'of a path:'),
    ('eq', 'energy',
     '*E* = *P*_{hover} *t* + *k*_{move} *d* + *c*_{leg} *n*_{legs},   '
     '*t*_{leg} = *L*/*v* + *v*/*a* + 0.06 s,'),
    ('p',
     'where every leg of length *L* is flown on a trapezoidal profile from '
     'rest to rest at *v* = 5 m/s and *a* = 3 m/s^{2}, and 0.06 s of '
     'settling is measured at each arrival. The time of a path is thus '
     'known exactly from the plan, which is what allows C1 to predict an '
     'energy before flying. The model was calibrated on 12 flights on '
     'fresh simulator instances, squares of 10 to 60 m with hovers of '
     'varied length to decorrelate time from distance, and checked on four '
     'validation flights never used in the fit, including an eight-leg '
     'zigzag ({tab:energy}). Without the leg term the error is structured '
     'and optimistic, up to 4.2 % on 10 m legs, because every leg pays the '
     'same two accelerations; with it the transit errors fall within '
     '±0.16 %. *E*_{land} is calibrated separately from twelve descents '
     'from 8 m (3199 J); the planner uses 3387 J, the 95th percentile of '
     'the landings commanded in missions, which include their pauses.'),
    ('tab', 'energy',
     'Energy model parameters, least-squares fit on 36 segments of 12 '
     'calibration flights, with 95 % confidence intervals.',
     ['Parameter', 'Value', 'Interpretation'],
     [['*P*_{hover}', '285.64 ± 0.12 W', 'power to stay airborne; 285.9 W '
       'in a pure hover'],
      ['*k*_{move}', '−0.400 ± 0.042 J/m', 'cruise at 5 m/s costs about '
       '2 W less than hover'],
      ['*c*_{leg}', '65.2 ± 1.3 J', 'the two accelerations of each leg'],
      ['*E*_{land} (calibrated)', '3199 J, s.d. 15 J', 'descent from 8 m '
       'and touchdown, 12 descents'],
      ['*E*_{land} (planning)', '3387 J', '95th percentile of the landings '
       'commanded in missions']],
     [3.6, 3.6, 8.8]),
    ('p',
     'Three caveats bound the claims. The intervals measure the model\'s '
     'lack of fit rather than noise, since the simulation repeats to '
     '0.02 %. The negative *k*_{move}, cruise slightly cheaper than '
     'hover, is measured but its origin in the rotor model was not '
     'established. And the model is valid only for the calibrated '
     'profile, which the trajectory executor therefore flies by '
     'construction: stop at every vertex, constant speed and '
     'acceleration, constant altitude.'),
    ('h2', '4.6 C1: Energy-Feasibility Filter and Monitor'),
    ('p',
     'C1 evaluates {eq:c1} for every frontier at every selection, and the '
     'margin {eq:margin} at every map cycle, about once per second. '
     '{fig:c1} shows one real decision. The terms are computed as '
     'follows. *E*_{go} is the energy of the planned path from the vehicle '
     'to the frontier viewpoint by {eq:energy}, plus a hover of duration '
     'τ for what the plan does not contain: the computation before the '
     'path starts and the wait at the frontier before the next command. '
     '*E*_{ret} is the energy of the path from the viewpoint to the '
     'nearest landable cell of any confirmed zone, the take-off point '
     'included, plus another τ. A single return field, a distance '
     'transform seeded on every landable cell of every confirmed zone, '
     'gives the return cost from all frontiers in one pass.'),
    ('fig', 'c1', NEW + 'r6_c1_decision.png', 16.0,
     'C1 on one real decision, 196 s into the W3 mission of {fig:film}. '
     '(a) 55 admissible frontiers (circles), 49 rejected (crosses), the '
     'target chosen by C2 (star) and its path. (b) *E*_{go} + *E*_{ret} + '
     '*E*_{land} of every frontier, sorted, against *B* − *R* = 15.9 kJ: '
     'a frontier of 467 unknown cells needing 21.3 kJ is rejected before '
     'any ranking.'),
    ('p',
     'The latency τ was first a constant of 2 s. On a 190 m area the '
     'decision time grew with the map up to 21.5 s, 301 s of hover went '
     'uncounted, and I1 was violated seven times with C1 active. τ is now '
     'measured at every decision and C1 charges the maximum of the last '
     'five latencies plus the largest recent increase; with 5 s of latency '
     'injected on purpose, the violations fell from 2, 2 and 1 per mission '
     'to 0, 0 and 0. A guarantee is no stronger than the worst term it '
     'omits.'),
    ('p',
     'Two approximations remain. The return from *f* is planned on the '
     'current map, whereas the map at *f* is unknowable; this is '
     'optimistic, and the reserve *R* covers it. And the registry and the '
     'energy may change during the transit, which the monitor addresses: '
     'while the vehicle flies to its target, {eq:c1} is re-evaluated for '
     'that target from the current position at every cycle, and if it '
     'fails the target is abandoned and the vehicle returns. It does not '
     'select another frontier, a conservative choice discussed in '
     'Section 7. {alg:cycle} shows how the filter, the monitor and the '
     'shrink handling of Section 4.4 fit into one decision cycle. When no '
     'frontier is admissible, exploration is over for energy reasons, '
     'which is a legitimate end of mission rather than an error.'),
    ('alg', 'cycle', 'One decision cycle of the exploration manager, run '
     'on every new map (about 1 Hz).',
     ['**Input:** map *H*, registry *Z*, energy left *B*, position *p*, '
      'reserve *R*, weight α, mission state, current target *f*\\*',
      '**Notation:** *E*_{tot}(*f*) = *E*_{go}(*p*, *f*) + *D*(*f*) + '
      '*E*_{land}, the energy to reach *f*, return from it and land'],
     [(0, '(*L*, *F*, *O*) ← Analyse(*H*)',
       'landable cells, frontiers, obstacles'),
      (0, '(*Z*, *lost*) ← UpdateRegistry(*Z*, *L*)',
       'match, confirm (I4), demote'),
      (0, '*D* ← ReturnField(*O*, *Z*)',
       'energy to the nearest confirmed zone'),
      (0, '*m* ← *B* − *R* − *E*_{land} − *D*(*p*)',
       'margin {eq:margin}, logged'),
      (0, '**if** *lost* ≠ ∅ **and** *m* < 0 **then**',
       'the registry shrank under I1'),
      (1, 'log a violation; FlyBack(*p*, *Z*)', ''),
      (0, '**if** state = NAVIGATING **and** *E*_{tot}(*f*\\*) > *B* − *R* '
       '**then**', 'C1 monitor'),
      (1, 'abandon *f*\\*; FlyBack(*p*, *Z*)', ''),
      (0, '**if** state = SELECTING **then**', ''),
      (1, '*A* ← {*f* ∈ *F* : *E*_{tot}(*f*) ≤ *B* − *R*}',
       'C1 filter {eq:c1}'),
      (1, '**if** *A* = ∅ **then** FlyBack(*p*, *Z*)',
       'no affordable frontier left'),
      (1, '**else**', ''),
      (2, 'rescale *G* and *M* to [0, 1] over *A*', 'as in {eq:utility}'),
      (2, '*f*\\* ← argmax_{f∈A} α *G̃*(*f*) + (1 − α) *M̃*(*f*)', 'C2'),
      (2, 'Navigate(*f*\\*)', ''),
      (0, '**if** state = RETURNING **then**', 'Section 4.8'),
      (1, '**if** a zone *z*′ is cheaper to reach by more than 541 J', ''),
      (2, '**and** I1 would hold without *z*′ **then** switch the return '
       'to *z*′', 'at most 3 times'),
      (1, '**if** arrived at the end of the return path **then** '
       'CommitLanding', 'irreversible')],
     ['FlyBack(*p*, *Z*): stop exploring and fly to the nearest confirmed '
      'zone, argmin_{z∈Z} *E*_{ret}(*p*, *z*) (state RETURNING).']),
    ('h2', '4.7 C2: Energy-Margin Utility'),
    ('p',
     'C1 is a filter and treats all admissible frontiers as equal. C2 '
     'ranks them by trading information against margin. For each '
     'admissible frontier the share of the usable budget it would leave '
     'is'),
    ('eq', 'share',
     'share(*f*) = clip((*B* − *R* − *E*_{tot}(*f*)) / (*B* − *R*), 0, 1),   '
     '*E*_{tot} = *E*_{go} + *E*_{ret} + *E*_{land},'),
    ('p', 'and the utility is'),
    ('eq', 'utility',
     '*U*(*f*) = α *G̃*(*f*) + (1 − α) *M̃*(*f*),   '
     '*x̃* = (*x* − min_{A} *x*) / (max_{A} *x* − min_{A} *x*),'),
    ('p',
     'where *G* is the number of unknown cells within the sensor radius '
     'of the viewpoint, *M* is the share, and both are rescaled to [0, 1] '
     'over the admissible set *A* of the current cycle. The rescaling of '
     'both terms is essential and was established by measurement. In the '
     'first version *G* was divided by its maximum and *M* was the share '
     'itself. Over 2225 decisions of 60 missions, *G* spread by 0.88 '
     'between candidates and *M* by only 0.12 (medians), so *G* dominated '
     'from α = 0.10 and 92 % of the decisions at α = 0.5 were already the '
     'pure-gain choice: α acted only on [0, 0.2]. With both terms rescaled '
     'per cycle, the choice leaves the pure-margin one at α = 0.35 and '
     'joins the pure-gain one at α = 0.55 (medians), and 52 % of the '
     'decisions at α = 0.5 differ from pure gain. The cost is that nearly '
     'equal margins are magnified, but only among frontiers C1 has already '
     'judged safe. At α = 1 the ranking is pure gain among admissible '
     'frontiers, configuration (b) of the proposal.'),
    ('h2', '4.8 Mission State Machine and Returns'),
    ('p',
     'The exploration manager is a state machine: INIT, TAKEOFF, '
     'SELECTING (frontiers, C1, C2), NAVIGATING (following the path while '
     'the margin is monitored), RETURNING (transit to the chosen zone), '
     'COMMIT_LANDING (descent engaged), LANDED and EMERGENCY. A mission '
     'ends on complete coverage, on no admissible frontier, on a target '
     'abandoned by the monitor, or on an external stop, and in all four '
     'the vehicle lands on a registry zone. Once the descent is engaged '
     'the decision is irreversible: oscillating between zones at the end '
     'of a mission is the worst behaviour energetically.'),
    ('p',
     'Three refinements were found necessary in flight. During a return '
     'the target may be switched to a confirmed zone that has become '
     'cheaper to reach by more than the cost of one stop and restart, '
     '*c*_{leg} + *P*_{hover} *v*/*a* = 541 J, at most three times per '
     'return, and only if I1 would still hold should the new zone be lost '
     'on arrival: the first version switched to zones that vanished one '
     'or two seconds after their confirmation, and one mission ended in '
     'its reserve. A frontier whose target cell turns out to be '
     'unreachable on approach (a roof measured higher up close) is '
     'blacklisted for 30 s, and for 300 s after three failures. And when '
     'the inflated obstacles enclose the vehicle, as over the rooftop '
     'units of W3, the return may use an escape map with a 1 m vertical '
     'margin instead of 2 m, never used to explore.'),
]
