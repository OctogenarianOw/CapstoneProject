"""
Report text, first half: title page, abstract, Sections 1 to 4.

Every number in this file comes from the experiment directories named
next to it in docs/prompt-maitre-projet-uav.md (sections 4, 7 and 10)
or from the figure index experiments/2026-10-24_thesis_figures/README.md.
"""

FIGS = 'experiments/2026-10-24_thesis_figures/'

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
     'explorer returning at a fixed 30 % battery threshold. The '
     'energy-margin utility shows an interior optimum: α = 0.75 gives the '
     'highest coverage on all three worlds, significantly above both pure '
     'margin and pure gain. A deliberately provoked loss of the zone in use '
     'during a return was detected within 3.1 to 4.1 s and answered at the '
     'same decision cycle; it also exposes the limit of the guarantee, '
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
     'runway constraints. For a UAV over unknown terrain the alternate is '
     'not filed before departure. It must be discovered during the flight, '
     'and its reachability must be re-verified as the battery drains and '
     'as the map, and therefore the alternate itself, is revised.'),
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
     'LiDAR returns are fused into a 2.5D elevation map; slope, roughness '
     'and step height are computed separately and tested against '
     'thresholds derived from the measured map noise; landable cells are '
     'grouped into zones held in a dynamic registry, in which a zone '
     'becomes usable only after every cell of a landing footprint has been '
     'observed on two distinct passes. The first contribution, C1, is an '
     'energy-feasibility invariant over this registry: a frontier is '
     'admissible only if the energy to reach it, plus the energy to return '
     'from it to the nearest confirmed zone and land, fits within the '
     'energy left minus a reserve. The invariant is also monitored at '
     'every map update, so that the loss of a zone triggers an immediate '
     're-evaluation rather than waiting for the next frontier selection. '
     'The second contribution, C2, ranks the admissible frontiers by '
     'U = αG + (1 − α)M, a convex combination of information gain and '
     'energy margin, both rescaled over the candidates of each decision so '
     'that α keeps the same meaning throughout a mission. Both mechanisms '
     'rest on an energy model calibrated against rotor shaft power, after '
     'the distance-proportional model of the original proposal was refuted '
     'by measurement.'),
    ('p',
     'The system was evaluated in PX4 software-in-the-loop simulation with '
     'Gazebo Harmonic on three worlds of increasing size and difficulty, '
     '210 missions in total, against nearest-frontier exploration without '
     'energy reasoning (B0) and a gain-driven explorer that returns at a '
     'fixed 30 % battery threshold (B1), the common practice in the '
     'literature. No mission with C1 violated the invariant (0 of 150), '
     'while B0 violated it in all 30 of its missions and B1 in 11 of 30: '
     'all ten on the tightest budget and none on the world where its '
     'threshold happens to exceed the return cost. The energy-margin '
     'utility has an interior optimum: α = 0.75 gives the highest coverage '
     'on all three worlds, significantly above both pure margin and pure '
     'gain. A deliberately provoked loss of the landing zone in use was '
     'detected 3.1 to 4.1 s after the obstruction appeared and answered at '
     'the same decision cycle; it also exposed the limit of the guarantee, '
     'since in three of four cases no other zone remained within the '
     'margin and the reserve covered the shortfall, at most 0.30 kJ.'),
    ('p',
     'The contributions of this report are therefore: (i) an '
     'energy-feasibility invariant enforced over a registry of landing '
     'zones that is built online and can shrink, with an explicit '
     'treatment of the bootstrap case and of zone loss; (ii) an '
     'energy-margin utility whose per-decision normalisation makes the '
     'trade-off parameter act over its whole range; and (iii) an empirical '
     'characterisation: a calibrated and validated energy model, a '
     '210-mission evaluation with criteria fixed before the flights, a '
     'deliberately provoked registry shrink, and a set of documented '
     'negative results. Section 2 reviews related work, Section 3 states '
     'the problem formally, Section 4 describes the method and Section 5 '
     'its implementation, Section 6 reports the experiments, and '
     'Section 7 concludes with short and long-term future work.'),

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
     'objective aware of localisation uncertainty, steering the robot away '
     'from regions where the map would be unreliable (Papachristos et al., '
     '2017), and of risk under stochastic dynamics (Han, 2023). These '
     'developments reflect a broader shift, articulated in the SLAM '
     'literature (Cadena et al., 2016), from map building as an end in '
     'itself towards perception in the service of a task.'),
    ('p',
     'The common thread is that the exploration objective remains '
     'information: where to look next in order to learn the most. Energy '
     'enters, when it does, as a path-length cost added to the utility, '
     'and the availability of a place to land, which is what actually '
     'bounds a battery-powered rotorcraft, plays no role in the decision. '
     'This work keeps the frontier paradigm precisely because its '
     'candidates are discrete and explicit: each one can be checked against '
     'an energy constraint before it is ranked, which is the structure C1 '
     'and C2 require.'),
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
     '(2012) demonstrated autonomous landing at unprepared sites with a '
     'full-scale helicopter that evaluated candidate sites from LiDAR '
     'during the approach, and Takahashi et al. (2013) benchmarked safe '
     'landing area determination algorithms against surveyed field sites, '
     'finding geometric hazard scores more dependable than texture-based '
     'ones. Visual SLAM such as ORB-SLAM3 (Campos et al., 2021) can now '
     'produce dense maps from cameras, but the absolute height accuracy '
     'needed for slope and step tests still favours ranging sensors.'),
    ('p',
     'Two conclusions follow. First, geometric tests on a 2.5D map are '
     'mature enough to be used as a building block rather than a research '
     'object, and this work treats them as such. Second, the literature '
     'treats landing detection as a perception problem: sites are assessed '
     'on a map built for another purpose, during a final approach, or '
     'offline. The result neither steers the exploration that builds the '
     'map nor is checked against the energy needed to reach the site. A '
     'third observation, which this work makes explicit in Section 6.2, is '
     'that detectors are usually evaluated on a single map, whereas the '
     'absence of dangerous false accepts is a joint property of the map '
     'and the detector, and must be measured over many maps.'),
    ('h2', '2.3 Terrain Mapping and Traversability'),
    ('p',
     'Elevation mapping with per-cell uncertainty is the standard '
     'representation for this purpose: each cell holds a height estimate '
     'and its variance, updated by a scalar Kalman filter (Kalman, 1960) '
     'as new returns arrive, with pose uncertainty propagated into the map '
     '(Fankhauser et al., 2014, 2018). Traversability analysis then '
     'derives local descriptors from the map. Beycimen et al. (2026) show '
     'that slope, roughness and step height discriminate safe from '
     'hazardous terrain in real time under field conditions, and Brubaker '
     'et al. (2013) established that surface roughness can be recovered '
     'from LiDAR returns. These techniques are reused here without '
     'modification. They matter to the contribution for a different '
     'reason: their error characteristics set the thresholds, the landing '
     'footprint and hence the safety margin of everything built on top. '
     'One known representational limit, a single height per cell that '
     'cannot represent two surfaces, turns out to be the dominant source '
     'of false refusals near obstacles in our largest world (Section 6.6).'),
    ('h2', '2.4 Energy-Constrained Planning and Reachability'),
    ('p',
     'Empirical studies of multirotor consumption show that power is '
     'dominated by the cost of staying airborne rather than by the '
     'distance flown (Liu et al., 2017; Abeywickrama et al., 2018). A '
     'distance-proportional cost is therefore a poor model for missions '
     'that include hovering, decision pauses and frequent stops, a point '
     'this work confirms by measurement (Section 4.5). Planning under a '
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
     'constraints and thereby guarantee that a safe state stays reachable, '
     'but the renewal sets are either known a priori or discovered '
     'incidentally rather than being an objective of the mission. Control '
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
     'unknown, static and contained in a square exploration area *A*. It '
     'carries a single downward-facing 3D LiDAR and is localised by GPS '
     'through the autopilot estimator; no SLAM is used, following the '
     'supervisor\'s advice in the first semester. It flies at a fixed '
     'cruise altitude *h* above the take-off point (8 m in all experiments) '
     'and has an energy budget *B*_{0}. The energy left at time *t* is '
     '*B*_{t}, and a fixed reserve *R* = 0.1 *B*_{0} is withheld from '
     'planning. There is no wind. The energy budget is a logical budget '
     'measured by integrating rotor shaft power in simulation '
     '(Section 4.5), not the autopilot\'s simulated battery.'),
    ('h2', '3.2 Map, Landable Cells and Registry'),
    ('p',
     'The terrain is represented by a 2.5D grid *M*_{t} with 1 m cells, '
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
     'where *Z* denotes the registry at the time of evaluation. The '
     'invariant I1 required by the research question is'),
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
    ('h2', '3.4 Objective, Bootstrap and Outputs'),
    ('p',
     'The objective is to maximise the fraction of *A* that is observed, '
     'subject to I1 whenever *Z*_{t} is non-empty, and to end every mission '
     'by landing on a confirmed zone, never wherever the vehicle happens '
     'to be. The bootstrap case, *t* = 0 with no confirmed zone, is '
     'resolved by registering the take-off point as a *provisional* zone: '
     'the vehicle has just left it, so it can land there. This makes I1 '
     'meaningful from the first cycle without a special case, and the flag '
     'lets the analysis distinguish missions that end on a discovered zone '
     'from those that end at the start. It departs from the proposal, '
     'which excluded the launch point by assumption; the consequence, '
     'that the take-off point is assumed to stay free for the whole '
     'mission, is declared as a limitation (Section 7). The outputs of a '
     'mission are the trajectory, the registry history, the per-cycle '
     'record of every candidate with its energy terms, and the landing '
     'site.'),

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
     'component holds the decision authority, so that C1 and C2 sit in one '
     'place and can be disabled cleanly for the baselines.'),
    ('fig', 'overview', FIGS + 'r1_method_overview.png', 16.0,
     'Method overview. Top: the perception chain from LiDAR scans to the '
     'registry of confirmed landing zones. Bottom: the decision chain from '
     'frontiers to flight. Dashed boxes are the contributions of this work '
     '(the dynamic registry, C1 and C2); solid boxes are established '
     'techniques reused as building blocks. The registry feeds C1 with the '
     'confirmed zones and the state machine with every change, which '
     'triggers the re-evaluation of the invariant described in Section '
     '4.4.'),
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
     '(0.02 m)^{2}. The floor is not cosmetic: without it the variance of '
     'a cell decays as 1/*n*, and after a few hundred observations the '
     'gain is so small that the cell is effectively frozen and cannot be '
     'corrected even by a much better observation taken from close range. '
     'In early versions this produced phantom trenches several metres deep '
     'at ramp edges, under cells that had accumulated tens of thousands of '
     'observations with an absurd confidence.'),
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
     'height difference between the cell and its neighbours. The plane fit '
     'matters: with 5 cm of noise between neighbouring cells a finite '
     'difference produces about 6° of spurious slope, which the fit '
     'reduces roughly threefold. Because the plane is removed before '
     'roughness is measured, roughness is independent of slope by '
     'construction: a smooth ramp reads as slope and not as roughness, and '
     'a rubble field as roughness and not as slope. Section 6.2 verifies '
     'this separation against ground truth.'),
    ('p',
     'A cell passes when its slope is at most 6°, its roughness at most '
     '0.05 m and its step at most 0.24 m, and its window is fully '
     'observed. The four tests are applied separately rather than combined '
     'into a weighted score as in the proposal: a weighted sum allows a '
     'very flat but stepped cell to compensate its step with its flatness, '
     'whereas each descriptor captures a distinct way of damaging a '
     'landing gear. The slope threshold is not a convention. It sits '
     'between the 95th percentile of the slope measured on truly flat '
     'ground (4.34°) and the 1st percentile measured on the 10° ramp of the '
     'validation world (8.25°), both on the reference map (Section 6.2).'),
    ('p',
     'A cell is *landable* only if every cell covered by a landing '
     'footprint of 1.5 m centred on it passes. The footprint element is '
     'an over-approximation of the disc: every cell the disc touches, even '
     'partially, must pass, which at 1 m resolution is the 3 × 3 square. An '
     'earlier element kept only the cells contained in the disc, a single '
     'cell for a 1.5 m footprint, and therefore accepted cells it had '
     'never tested; correcting it reduced the dangerous false accepts on '
     'the reference map from 25 to 0. The footprint was then checked '
     'against the measured landing precision (Section 6.3): the vehicle '
     'needs 1.13 m, but a 1.2 m footprint, whose element loses the '
     'diagonal cells, reintroduced 2 dangerous false accepts on the '
     'reference map. The diagonal cells are a margin that the map error '
     'requires, and 1.5 m was kept. With an analysis radius *r*_{a} and a '
     'footprint radius *r*_{f} in cells, an isolated zone must be at least '
     '2(*r*_{a} + *r*_{f}) + 1 cells wide to be detected, 5 m in this '
     'configuration. The real lever for detecting smaller zones is '
     'therefore the map error, which costs energy to reduce (flying lower, '
     'passing more often); this is exactly the trade-off that C2 '
     'formalises.'),
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
     'counting many passes. The rule replaced an earlier one that counted '
     'passes over the zone\'s centroid: offline on the validation world, '
     'that rule confirmed zones of which only 538 m^{2} out of 2465 m^{2} '
     'had in fact been seen twice. A single view from one angle confirms '
     'nothing; a second pass from another position is what removes the '
     'artefacts of oblique observation. Once confirmed, a zone is demoted '
     'or retired when an update refuses enough of its cells that no whole '
     'footprint remains.'),
    ('p',
     'The registry is what makes the safe set dynamic, and its shrinking '
     'is the case that no prior work on energy-constrained exploration '
     'treats. When an update removes a confirmed zone, the system (1) '
     'detects the event at the registry update rather than at the next '
     'frontier selection; (2) re-evaluates I1 immediately against the '
     'reduced registry; (3) if I1 still holds through another zone, '
     'continues and logs the event; (4) if it no longer holds, interrupts '
     'the current navigation and returns to the nearest reachable zone; '
     'and (5) if no zone is reachable within the guarantee, logs a '
     'violation and still flies to the nearest reachable zone, the '
     'reserve covering the shortfall. Section 6.6 provokes this sequence '
     'deliberately.'),
    ('h2', '4.5 Energy Model'),
    ('p',
     'The proposal planned a model *E* = *k d*, calibrated from the '
     'battery telemetry of the PX4 simulator. Both parts were refuted. The '
     'simulated battery reports a current of −1 A, a discharged capacity '
     'of zero, and a remaining fraction that decreases linearly with the '
     'time the vehicle has been armed: regressing it on distance would only '
     'measure flight duration. The signal used instead is the mechanical '
     'shaft power of the four rotors, computed from the motor speed '
     'commands that PX4 sends to the simulator. The instrument was '
     'validated in hover, where the analytic prediction from weight and '
     'thrust coefficients (806.0 rad/s, 286.5 W) falls within 0.2 % of the '
     'measurement (805.5 rad/s, 285.9 W). Fitted to a survey flight, the '
     'distance model gives *k* = 48 J/m and predicts 77 J for a 60 s hover '
     'that costs over 17 kJ: flight time explains 87 % of the survey '
     'energy. A multirotor spends most of its energy staying in the air, '
     'which had to be measured rather than assumed.'),
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
     'energy before flying. The model was calibrated on 16 flights on '
     'fresh simulator instances: squares of 10, 20, 40 and 60 m flown three '
     'times each, with a hover of 10, 20 or 30 s before each lap to '
     'decorrelate time from distance, and four validation flights never '
     'used in the fit, including an eight-leg zigzag. {tab:energy} gives '
     'the parameters. Without the leg term the model already passes the '
     '10 % acceptance bar by a factor of ten, but its error is structured '
     'and optimistic, up to 4.2 % on 10 m legs, because every leg pays the '
     'same two accelerations; with it the transit errors fall within '
     '±0.16 %. The zigzag, with twice as many legs per metre as the '
     'squares, confirms that the cost is per leg. *E*_{land} is calibrated '
     'separately from twelve descents from 8 m (3199 J, standard deviation '
     '15 J); the planner uses 3387 J, the 95th percentile of the landings '
     'commanded by the mission manager, which include its own pauses.'),
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
     '{fig:c1} illustrates one decision. The terms are computed as '
     'follows. *E*_{go} is the energy of the planned path from the vehicle '
     'to the frontier viewpoint by {eq:energy}, plus a hover of duration '
     'τ for everything the plan does not contain: the computation before '
     'the path starts, the settling, and the wait and computation at the '
     'frontier before the next command. *E*_{ret} is the energy of the '
     'path from the viewpoint to the nearest landable cell of any '
     'confirmed zone, the provisional take-off point included, plus '
     'another τ, since the return also starts one decision after the '
     'energy is read. A single return field, a distance transform seeded '
     'on every landable cell of every confirmed zone, gives the return cost '
     'from all frontiers in one pass.'),
    ('fig', 'c1', FIGS + 'r2_c1_geometry.png', 12.5,
     'The admissibility rule of C1 on one decision, seen from above, with '
     'illustrative energies. Frontier f1 can be reached and the vehicle '
     'can still return from it to the confirmed zone z1 and land within '
     '*B* − *R*: it is admissible and passed to C2. Frontier f2 costs '
     'about as much to reach, but its nearest zone is the provisional '
     'take-off point, far behind the vehicle, and the sum exceeds the '
     'budget: C1 rejects it before any ranking, whatever its information '
     'gain.'),
    ('p',
     'The latency τ was first a constant of 2 s, which sufficed while '
     'decisions took under a second. On a 190 m area the decision time '
     'grew with the map up to 21.5 s, 301 s of hover went uncounted, and '
     'I1 was violated seven times with C1 active. τ is now measured at '
     'every decision and C1 charges the maximum of the last five '
     'latencies plus the largest recent increase; with 5 s of latency '
     'injected on purpose, the violations fell from 2, 2 and 1 per mission '
     'to 0, 0 and 0. A guarantee is no stronger than the worst term it '
     'omits.'),
    ('p',
     'Two approximations remain. The return from *f* is planned on the '
     'current map, whereas the map at *f* is unknowable; this is '
     'optimistic, and the reserve *R* covers it. And the registry may '
     'change during the transit, which the monitor addresses: '
     '{alg:cycle} shows how the filter, the monitor and the shrink '
     'handling of Section 4.4 fit into one decision cycle. When no '
     'frontier is admissible, exploration is over for energy reasons, '
     'which is a legitimate end of mission rather than an error, and the '
     'vehicle returns.'),
    ('alg', 'cycle', 'One decision cycle of the exploration manager, run '
     'on every new map (about 1 Hz).',
     ['input: map M, registry Z, energy left B, position p, reserve R, α',
      'W ← analyse(M)        ▷ landable cells, frontiers F, obstacles',
      'Z, lost ← update_registry(Z, W, passes)   ▷ match, confirm (I4), '
      'demote',
      'D ← return_field(W, Z)      ▷ cost to the nearest confirmed zone',
      'm ← B − R − E_land − D(p)   ▷ margin (1), logged every cycle',
      'if lost ≠ ∅ and m < 0 then  ▷ registry shrank under the guarantee',
      '    log violation; RETURN to argmin_z E_ret(p, z)',
      'if state = SELECTING then',
      '    A ← {f ∈ F : E_go(p, f) + D(f) + E_land ≤ B − R}   ▷ C1, (3)',
      '    if A = ∅ then RETURN to argmin_z E_ret(p, z)',
      '    else rescale G, M over A; f* ← argmax_A αG + (1 − α)M   ▷ C2',
      '         NAVIGATE to f*',
      'if state = RETURNING and a zone is cheaper by > 541 J',
      '        and I1 holds without it then switch target   ▷ ≤ 3 per return',
      'if arrived over a landable cell of the target then COMMIT landing']),
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
     'decisions at α = 0.5 differ from pure gain. The cost is that when '
     'all margins are nearly equal a small difference is magnified, but '
     'only among frontiers that C1 has already judged safe. At α = 1 the '
     'ranking is pure information gain among admissible frontiers, which '
     'is exactly configuration (b) of the proposal; at α = 0 the vehicle '
     'always takes the frontier that leaves the most energy, which in '
     'practice keeps it close to its landing zones.'),
    ('h2', '4.8 Mission State Machine and Returns'),
    ('p',
     'The exploration manager is a state machine: INIT, TAKEOFF, '
     'SELECTING (frontiers, C1, C2), NAVIGATING (following the path while '
     'the margin is monitored), RETURNING (transit to the chosen zone), '
     'COMMIT_LANDING (descent engaged), LANDED and EMERGENCY. A mission '
     'ends in one of three ways, each logged separately: complete '
     'coverage (no valid frontier), budget exhausted (no admissible '
     'frontier), or an external stop. In all three the vehicle lands on a '
     'registry zone, never where it stands. Once the descent is engaged '
     'the decision is irreversible, since oscillating between zones at the '
     'end of a mission is the worst behaviour energetically.'),
    ('p',
     'Two refinements of the return were found necessary in flight. '
     'First, during a return the target may be switched to a confirmed '
     'zone that has become cheaper to reach by more than the cost of one '
     'stop and restart, *c*_{leg} + *P*_{hover} *v*/*a* = 541 J, at most '
     'three times per return. The first version of this rule switched to '
     'zones confirmed in the same cycle that vanished one or two seconds '
     'later, and one mission ended in its reserve; a switch is therefore '
     'taken only if I1 would still hold should the new zone be lost on '
     'arrival. Second, a frontier whose target cell turns out to be '
     'unreachable on approach (for instance a roof measured higher up '
     'close) is blacklisted for 30 s, and for 300 s after three failures, '
     'and a new frontier is selected. Finally, when the inflated obstacles '
     'of the nominal map enclose the vehicle, which happened over the '
     'rooftop units of the largest world, the return may use an escape '
     'map with a 1 m vertical margin instead of 2 m; the escape map is '
     'never used to explore.'),
]
