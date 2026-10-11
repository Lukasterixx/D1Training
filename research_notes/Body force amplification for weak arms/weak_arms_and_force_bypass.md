# Weak arms on legged robots, and the ways prior work applies more force than the arm can

Scope: low-cost or weak arms on small quadrupeds (2020-2026), and every distinct strategy prior work uses to apply forces beyond a small arm's rating. Each strategy is set against the student's case: Go2 (~15 kg) + Unitree D1 (published 3.3 N·m on J0/J1, 1.7 N·m on J2-J5); a spring-loaded combiner-box lever; ~1.15 N·m from arm-only pushing in simulation; gripping and pulling fails at 0.5 N·m because the grip slips; ~1-1.5 N·m for "crouch with the arm held rigid" in a static model.
Search date: 2026-10-01. Numbers are quoted as the sources state them. Where a paper reports no force numbers, the notes say so. "arXiv-only" means no peer-reviewed venue was found for it.

## 1. Platforms: low-cost arms on small quadrupeds, their rated payloads and torques, and whether the Unitree D1 appears in published work

### Takeaway
Most low-cost legged manipulators in 2022-2026 pair a Unitree Go1/Go2 with an arm rated at 0.25-1.5 kg payload (WidowX 250, ARX5, Piper, Airbot Play). Heavier labs use the B1/B2 with a Z1 (2-5 kg payload, 33 N·m joints). The D1 is the weakest arm in this set: 500 g payload, with no joint torque in Unitree's own product pages. I found exactly one paper that uses the D1, a 2026 arXiv preprint, and it reports no force numbers. A 2025 Caltech student report states that no robotics papers used the D1, that Unitree had stopped selling it by March 2025, and that its own unit failed. Only one platform paper measures force that the whole body adds beyond the arm's rating: Portela et al., ICRA 2024, with a 90 N pull against a 36 N rated payload.

### Cited Findings
**Unitree D1 (the student's arm)**
- D1/D1-T published specs: payload "500g"; weight "About 2.37Kg"; reach "550mm (Gripper not included), 670mm (Gripper included)"; "Bus Servo"; "6 Axis + 1 Gripper"; "24V 2.5A (MAX 5A)"; "60W". The page claims "position control, velocity control, and force control" and an "Ultra-Precision Force-Controlled Digital Servo Motor", but gives **no joint torque values** — [Unitree D1-T page](https://www.unitree.com/mobile/D1-T/)
- Retailer listing: joint ranges J1 ±135°, J2 ±90°, J3 ±90°, J4 ±135°, J5 ±90°, J6 ±135°; max joint speed 90°/s; payload 500 g — [RobotShop D1 listing](https://www.robotshop.com/products/unitree-go2-servo-robotic-arm-d1)
- The project's own motor model uses "Unitree D1-550 published joint torques (README, 'The arm's mass'): first two joints 3.3 N·m, last four 1.7". The project README records that Unitree publishes 3152 g for the D1-550, against "About 2.37 Kg" on the D1-T page. The two figures may describe different variants or include different parts — [project motor_model.py](../../motor_model.py); [Unitree D1-T page](https://www.unitree.com/mobile/D1-T/)
- A search-engine summary claimed a D1 "maximum joint torque of 23.5N·m". **I could not find that number on any page I fetched**, and the STEMfinity page it seemed to come from does not contain it. Treat it as unverified and probably wrong — [STEMfinity D1 page](https://stemfinity.com/products/d1-servo-arm)
- Unitree's "Brushless Digital Servo" page lists a J288 (stall "1.5Nm") and an S288 (stall "0.6Nm"). It does **not** say which arm uses them, so do not attribute them to the D1 — [Unitree Digital Servo](https://www.unitree.com/mobile/DigitalServo/)
- **Only published D1 paper found:** Zhou, Zhou, Luu, Han, Zhang, Huang, Li, Ajoudani, Xu, She, "Learning Tactile-Aware Quadrupedal Loco-Manipulation Policies" (arXiv 2604.27224, v3 July 2026, arXiv-only). Hardware: "12-DOF Unitree Go2 quadruped robot and a 6-DOF D1 robot arm, both powered by the battery of Go2", with a UMI gripper carrying FlexiTac tactile arrays (32×12 taxels) and a GoPro. The paper states "the Unitree D1 arm only accepts joint-position commands at 10 Hz". Tasks: marker reorientation using extrinsic contact, socket insertion, **valve tightening with cyclic grasp-twist-release**, and delicate objects. **No payload, joint torque, gripper force or valve torque numbers are reported** — [arXiv 2604.27224](https://arxiv.org/html/2604.27224v3)
- Caltech AMBER Lab SURF report (Zeng, Olkin, Ames, 2025; student report, not peer reviewed): "Unitree no longer sold the arm by March 2025. There are no robotics papers that use the arm". Unitree gives 10 Hz as the arm's control cycle. Their D1 "fails to achieve the commanded servo positions and does not follow any commands after a few minutes of execution", and "A few servomotors were burning hot". They concluded "the arm is defective" — [Caltech SURF report](https://eloisezeng.people.caltech.edu/documents/32338/2025_SURF_Final_Report.pdf)

**Other arms (manufacturer or retailer specs)**
- Unitree Z1: "maximum torque of 33 Nm" per joint, harmonic reducers; Z1 Air payload 2 kg; Z1 Pro payload 3-5 kg; reach 740 mm — [ROS Components Z1](https://www.roscomponents.com/product/z1-arm/); [RobotShop Z1 Air](https://www.robotshop.com/products/unitree-z1-air-robot-arm). Portela et al. give the Z1 as "5.3 kg", reach "0.74 m", "rated payload of 36 N" — [arXiv 2405.01402](https://arxiv.org/html/2405.01402)
- ARX5: weight "3.4kg", reach "620mm", payload "1.5kg", planetary gear motors. UMI-on-Legs rejected the Z1 because "it weighs 5kg, making it too heavy for Go2", and rejected the ViperX for insufficient payload — [UMI on Legs hardware notes](https://github.com/real-stanford/umi-on-legs/blob/main/real-wbc/docs/hardware_design_choices.md)
- Interbotix WidowX 250 6DOF: "Working Payload: 250g", recommended at no more than 50% extension; reach 650 mm; seven DYNAMIXEL XM430-W350-T plus two XL430-W250-T servos — [Trossen docs](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html)
- AgileX PiPER: payload 1.5 kg, weight 4.2 kg, reach 626 mm; no joint torques in the listings — [Generation Robots PiPER](https://www.generationrobots.com/en/404258-6-axis-robotic-arm-piper.html)
- AIRBOT Play: max payload 3.5 kg, "stable" payload 1.5 kg, weight 3.5 kg, reach 647 mm — [AIRBOT docs](https://discover-robotics.github.io/docs/latest/AIRBOT-Play/manual/spec/)
- Spot Arm (the high-end reference point): "Up to 11 kg" lift; "Continuous lift capacity at 0.5 m extension: 5 kg"; drag "Up to 25 kg" on carpet; "Peak clamp force (at tip of opening): 130 N"; mass "8 kg"; length "984 mm" — [Spot SDK arm spec](https://dev.bostondynamics.com/docs/concepts/arm/arm_specification.html)

**Papers using low-cost arms on quadrupeds**
- Deep Whole-Body Control (Fu, Cheng, Pathak, CoRL 2022): Go1 + "6DoF Interbotix WidowX 250s"; a unified RL policy for legs and arm — [arXiv 2210.10044](https://arxiv.org/abs/2210.10044); [GitHub](https://github.com/MarkFzp/Deep-Whole-Body-Control)
- RoboDuet (Pan, Ben, et al.; RA-L 2025 per a citing search result): Go1 Edu (12 kg) + ARX5 (3.35 kg), with cross-embodiment deployment — [arXiv 2403.17367](https://arxiv.org/html/2403.17367v4)
- UMI on Legs (Ha, Gao, Fu, Tan, Song, CoRL 2024): Go2 + ARX5. Tasks: tossing, pushing a "10lbs kettlebell", cup rearrangement; >70% success on all; system cost "approximately $24,000". One fetched summary gave the venue as RSS 2024; CoRL 2024 is the venue the authors' listing supports — [arXiv 2407.10353](https://arxiv.org/html/2407.10353v1); [project](https://umi-on-legs.github.io/); [Semantic Scholar](https://www.semanticscholar.org/paper/UMI-on-Legs:-Making-Manipulation-Policies-Mobile-Ha-Gao/5f146b8945fac10b1ed6c06f60c3541d8bf5449f)
- MLM (arXiv 2508.10538): Go2 + 6-DoF Airbot Play — [arXiv 2508.10538](https://arxiv.org/html/2508.10538)
- NMPC framework with non-negligible arm dynamics (arXiv 2507.22042): Go2 + "4-DoF Kinova arm" — [arXiv 2507.22042](https://arxiv.org/abs/2507.22042)
- TAC-LOCO (Hu, Zhou, Malik, Lin, Lee, She, Gu; arXiv 2607.10132, 2026, arXiv-only): Go2 + WidowX 250 + tactile gripper; "47% reduction in grasping force" against baselines; "object drop rate of less than 1%". This is the closest prior work on grip slip under load changes — [arXiv 2607.10132](https://arxiv.org/abs/2607.10132)
- Learning Force Control for Legged Manipulation (Portela, Margolis, Ji, Agrawal, ICRA 2024): B1 ("55 kg") + Z1 ("5.3 kg"). Commanded force −70 to 70 N per axis; real tracking error "within the range of 5–10 N" against a dynamometer. The policy produced a **"90 N" pull, above the arm's "rated payload of 36 N"**, by coordinating body and arm. Gravity compensation up to "4 kg" payload. No force sensor is used — [arXiv 2405.01402](https://arxiv.org/html/2405.01402); [IEEE](https://ieeexplore.ieee.org/document/10611066/)
- UniFP (Zhi, Li, Yin, Jia, Huang; arXiv 2505.20829, v2 Oct 2025): B2 + Z1 (and a G1 humanoid). Training commands "[-60 N, 60 N]" for the end-effector force **and for a separate base force**; hardware errors "within 10 N" over 0-60 N; about 39.5% higher success than position-only baselines on wipe, open/close cabinet and open drawer — [arXiv 2505.20829](https://arxiv.org/html/2505.20829)
- Visual Whole-Body Control (VBC): B1 + Z1 — [arXiv 2403.16967](https://arxiv.org/html/2403.16967v5)
- Rigo, Hu, Gupta, Nguyen, "Hierarchical Optimization-based Control for Whole-body Loco-manipulation of Heavy Objects" (ICRA 2024 per a citing search result): Aliengo + "custom-made robotic arm"; lifts and carries an "8kg payload"; also manipulates doors — [arXiv 2311.00112](https://arxiv.org/abs/2311.00112)
- RoLoMa (Ferrolho, Ivan, Merkt, Havoutis, Vijayakumar; Autonomous Robots 2023): ANYmal B + Kinova arm. Results are under section 6 — [arXiv 2203.01446](https://arxiv.org/pdf/2203.01446); [Springer](https://link.springer.com/article/10.1007/s10514-023-10146-0)
- Calf-Integrated Arms (Pan, Ren, Chan, Sun, Zhou; arXiv 2607.06186, 2026, arXiv-only): Go2 with a manipulator (prismatic slider, two revolute joints, gripper) built into each front calf; cooperative two-handed lift; the abstract gives no mass, torque or payload — [arXiv 2607.06186](https://arxiv.org/abs/2607.06186)

### Inferences
- The D1 is roughly an order of magnitude weaker than the Z1 in joint torque (3.3 vs 33 N·m peak) and 3× below the ARX5/PiPER in payload. No published work characterises what a D1 can push or pull. Any measured D1 force number in the thesis would be new data, but it must be labelled simulation-only until measured on hardware.
- Portela et al. (90 N vs 36 N rated, about 2.5×) is the strongest published evidence that a whole-body RL policy delivers end-effector force above the arm's rating. Their arm is a 33 N·m Z1 on a 55 kg B1. Body weight and arm strength are both about 10× the Go2+D1 case. The amplification ratio may not transfer when the arm's joints, not the body, set the limit. This is the student's "crouching still passes through the shoulder and elbow" problem.
- The D1's 10 Hz joint-position-only interface (confirmed by two independent sources) means a sim policy that relies on fast torque-level force regulation will not deploy as-is. Strategies that need no fast arm force control favour hardware transfer: a pre-attached tool, bracing, or force applied by legs or body.

### Gaps
- No Unitree page I fetched gives the D1's joint torques, stall torques or servo model. The 3.3/1.7 N·m values rest on the D1-550 README the project already cites; I did not re-fetch that README.
- The 2.37 kg vs 3152 g mass discrepancy between the D1-T page and the D1-550 README was not resolved.
- I found no published gripper force for the D1, ARX5, PiPER or WidowX gripper.
- I did not confirm venues for MLM, the NMPC paper, VBC or UniFP beyond arXiv.

## 2. Legs as manipulators and body-based (non-prehensile) manipulation: what forces do they achieve compared with an arm?

### Takeaway
Leg-based manipulation (Legs as Manipulator, Pedipulate, LocoMan, He et al., Go2 bipedal) has demonstrated door opening, button pressing, dishwasher closing and pushing. Almost none of it reports interaction forces. Pedipulate gives a >2.0 kg foot load and 2.6 kg box pushing. Body-based manipulation gives the large numbers: a 27 kg Raibo moving a 19.2 kg drum with its body; Spot moving 15-20 kg objects, above its 11 kg arm lift, using arm, torso and legs together; the RoLoMa ANYmal lifting 4.1 kg until the handle slipped out of the gripper. Whole-body pushing with a low-cost arm (UMI on Legs) shows the policy learning to lean in for more force.

### Cited Findings
**Legs as manipulators**
- Legs as Manipulator (Cheng, Kumar, Pathak, ICRA 2023): Go1 uses its front legs to climb walls, press buttons and kick a soccer ball. Skills are split into locomotion and "manipulation, involving using one leg to interact while balancing on the other three legs". **No force numbers** in the sources I read — [arXiv 2303.11330](https://arxiv.org/pdf/2303.11330); [The Robot Report](https://www.therobotreport.com/researchers-taught-quadruped-to-use-legs-as-manipulators/)
- Pedipulate (Arm, Mittal, Kolvenbach, Hutter, ICRA 2024): ANYmal D. Load carrying of "more than 2.0 kg" at the foot (a "2.0 kg backpack"); pushing a "box with a mass of 2.6 kg"; base disturbances "up to 132 N". The fetched paper text gives foot disturbances "up to 12 N". A search summary instead said "forces up to 130N on the foot", so the 12 N vs 130 N foot figure is **unresolved** (130 N may be the base figure misattributed). Door: the operator used "the joystick interface to push down the handle and then set a target point behind the door ... while simultaneously pushing the door open"; the push door had "a resistive spring-damper element at the hinge". A fridge pull door opened by foot friction alone. **No handle torque reported** — [arXiv 2402.10837](https://arxiv.org/html/2402.10837); [Semantic Scholar](https://www.semanticscholar.org/paper/Pedipulate:-Enabling-Manipulation-Skills-using-a-Arm-Mittal/45a832f6602396ca835356d365e82500454da21c)
- He, Lei, Ze, Sreenath, Li, Xu, "Learning Visual Quadrupedal Loco-Manipulation from Demonstrations" (IROS 2024): manipulation "using only its legs", including "lifting a basket ..., closing a dishwasher, pressing a button, and pushing a door". **No force numbers** — [arXiv 2403.20328](https://arxiv.org/abs/2403.20328)
- LocoMan (Lin et al., IROS 2024): two 3-DoF leg-mounted "loco-manipulators" on Go1 calves, "147 g" and "$370" each, Dynamixel servos. Opens drawers and a cabinet swing door, inserts a charger, pours. **No payload or force measurements** reported — [arXiv 2403.18197](https://arxiv.org/html/2403.18197); [GitHub (IROS'24)](https://github.com/linchangyi1/LocoMan)
- Bipedalism for Quadrupedal Robots (Zhang, Corcodel, Zhao, Humanoids 2025): Go2 stands on its hind legs and uses its front legs for "cart pushing, obstacle probing, and payload transport". **No numbers in the abstract** — [arXiv 2507.20382](https://arxiv.org/abs/2507.20382)
- Helpful DoggyBot (Wu, Fu, Cheng, Wang, Finn; arXiv 2410.00231, 2024): Go2 with a front-mounted, 3D-printed gripper driven by one Dynamixel XM430-W350-T (the WidowX's servo). It climbs a bed or couch and tilts its whole body to reach objects; 60% success on fetching after climbing a queen bed. **No force numbers** — [arXiv 2410.00231](https://arxiv.org/pdf/2410.00231)

**Body-based and whole-body force**
- Jeon, Jung, Choi, Kim, Hwangbo, "Learning Whole-body Manipulation for Quadrupedal Robot" (RA-L 2023): the robot ("Raibo", 27 kg) manipulates "a 19.2 kg water-filled drum" and "a 15.3 kg plastic box". "93.6 %" success in re-positioning and re-orienting objects within 0.03 m and 5°. The abstract does not say which body parts make contact — [arXiv 2308.16820](https://arxiv.org/abs/2308.16820)
- Multi-agent pushing (Go1s; ICRA 2025 per the search listing): each Go1 has "approximate payload capacity of 5 kg". Objects: "a 4 kg cuboid, a 3 kg T-shaped block, and a 10 kg cylinder" (the cylinder in simulation), with up to four agents; real Push-Cuboid and Push-T on Go1s — [arXiv 2411.07104](https://arxiv.org/html/2411.07104v2)
- UMI on Legs (CoRL 2024), Go2 + ARX5 pushing a 10 lb kettlebell: the whole-body controller learns to "lean forward in order to exert more force" when tracking error grows. Against static friction it shows "a wiggling motion along with increased pushing force until static friction breaks into a dynamic slide", then "catches itself by dropping its front feet forwards" — [arXiv 2407.10353](https://arxiv.org/html/2407.10353v1); [project](https://umi-on-legs.github.io/)
- Sumo (Zhang et al., MIT / RAI Institute / Cornell; arXiv 2604.08508, April 2026, arXiv-only), on Spot, whose arm has a "maximum lifting capacity of 11kg":
  - Objects: tire "15kg", crowd barrier "16kg", chair "16.5kg", box "20kg" pushed. Tire upright uses a "combination of its arm, torso, and legs"; front legs can be enabled for multi-limb manipulation.
  - Hardware success: Tire Upright 10/10, Barrier Drag 9/10, Box Push 10/10, and others 8-9/10.
  - The RAI blog: the 15 kg tire is "well above the robot's peak lift capacity of 11 kg", and the tire upright averages 5.9 s.
  - [arXiv 2604.08508](https://arxiv.org/html/2604.08508v1); [RAI blog](https://rai-inst.com/resources/blog/combining-sampling-and-learning-for-dynamic-whole-body-manipulation/)
- ReLIC (Zhu et al., RAI Institute; arXiv 2506.07876, 2025): dynamically "assigns each limb for manipulation or locomotion"; 12 real tasks at 78.9% average success. **No force numbers in the abstract** — [arXiv 2506.07876](https://arxiv.org/abs/2506.07876)
- Gong et al., "Legged Robots for Object Manipulation: A Review" (Frontiers in Mechanical Engineering 2023):
  - Gives the pushing condition "Fpush ≥ µW".
  - "leaning puts some of the robot's weight onto the pushed object, increasing and stabilizing the available contact force", with Inoue et al. (2010) as the example: a hexapod that "leans against a large block while holding it with two legs to push it".
  - "Kicking imparts a quick impulse rather than a sustained force".
  - [arXiv 2303.16865](https://arxiv.org/pdf/2303.16865); [Frontiers](https://www.frontiersin.org/journals/mechanical-engineering/articles/10.3389/fmech.2023.1142421/xml)

### Inferences
- Body pushing gets its force from the legs and from body weight, not from arm joints. That is why it scales to objects near or above the robot's own mass (19.2 kg vs 27 kg for Jeon; 20 kg vs Spot). For the combiner-box lever, a contact made directly by the body, or by a leg or foot, avoids the shoulder and elbow bottleneck that limits "crouch with a rigid arm". A Go2 front foot pressing a lever handle down is the closest analogue to Pedipulate's door-handle push. No paper reports the handle torque achieved that way.
- Leg-manipulation papers almost never report forces. A thesis that measures lever torque for arm-only, crouch-with-arm, foot or body, and tool strategies on the same handle would fill a quantitative gap. This is an inference from the absence of such numbers in the papers above.

### Gaps
- No leg-manipulation paper I read reports the force or torque a quadruped foot delivers to a door handle or lever.
- Whether Jeon et al. use the head, torso or legs could not be confirmed from the abstract.
- Go2-specific leg force numbers (e.g. the Go2 bipedal cart push) were not available in abstracts.

## 3. Tools for mechanical advantage, pre-attached or passive tools, and tool-conditioned policies

### Takeaway
The most direct formal precedent is Holladay, Lozano-Pérez and Rodriguez (IJRR). Their planner explicitly models torque and friction limits along a "forceful kinematic chain". To satisfy those limits it chooses among a grasped spanner, a pusher tool, a vise, environmental friction, a second arm or a weight. On legged robots, the closest "pre-attached tool" precedent is ETH's hook end-effector on ANYmal for door handles of 0-3 N·m resistance (CoRL 2024). Spot's door service also has a push mode that uses the wrist. Pulley blocks have been used to cut the load each arm bears (Hayakawa et al.). I found **no learning paper on tool-conditioned policies aimed at force or torque amplification**, and no legged robot that carries a lever or wrench to multiply arm torque.

### Cited Findings
- Holladay, Lozano-Pérez, Rodriguez, "Robust Planning for Multi-stage Forceful Manipulation" (IJRR; arXiv 2208.00319, revised Nov 2023):
  - The planner is augmented "with constraints that explicitly consider torque and frictional limits, captured through the proposed forceful kinematic chain constraint".
  - Childproof bottle: fixturing "via frictional contact with a surface, where the surface is either the table or a high-friction rubber mat", a vise, or a second robot arm. The cap is pushed and twisted by direct grasp, palm, fingertip, or "grasped pusher tool". When table friction is insufficient, the robot moves the bottle onto a higher-friction mat.
  - Nut twisting: the wrench is applied "through a grasp or through a grasped tool, i.e. a spanner". The beam is fixtured by "Another robot" or by "weighing down the beam with a large mass", with intermediate masses of 2.6-4.4 kg chosen as a robustness trade-off.
  - Fixed-base, not legged.
  - [arXiv 2208.00319](https://arxiv.org/html/2208.00319)
- Zhang et al., "Learning to Open and Traverse Doors with a Legged Manipulator" (CoRL 2024, PMLR v270): ANYmal with an arm and a **hook end-effector** for door handles. Simulated handle "resistance torque: Sampled uniformly from [0,3] Nm"; max handle angle [15, 90]°; door mass [15, 75] kg; hinge resistance [0, 30] N·m. "Many door handles have slippery surfaces which are especially hard to turn with a hook end-effector." Real-world success "95.0%" (20/20 pull, 18/20 push); performance degraded above the trained 30 N·m hinge resistance — [arXiv 2409.04882](https://arxiv.org/html/2409.04882v1); [PMLR](https://proceedings.mlr.press/v270/zhang25g.html)
- Spot Door Service "AutoPush" mode, for push bars, crash bars and unlatched doors: "the robot points the hand down and pushes the door open with its wrist". AutoGrasp grabs the handle, opens the door and walks through — [Spot SDK arm door example](https://dev.bostondynamics.com/python/examples/arm_door/readme)
- Hayakawa, Wan, Koyama, Harada, "A Dual-arm Robot that Autonomously Lifts Up and Tumbles Heavy Plates Using Crane Pulley Blocks" (arXiv 2101.09526, 2021): "crane pulley blocks reduce the payload that each robotic arm needs to bear". The abstract gives no masses — [arXiv 2101.09526](https://arxiv.org/abs/2101.09526)
- RoboTool (Xu et al., CMU and Google DeepMind; NeurIPS 2023 workshop per mlanthology): LLM-planned creative tool use on an arm and a quadruped, e.g. a surfboard as a bridge. The tools are not chosen for force amplification — [arXiv 2310.13065](https://arxiv.org/abs/2310.13065); [mlanthology](https://mlanthology.org/neuripsw/2023/xu2023neuripsw-creative/)
- Dong, Zhang, Calinon, Pokorny, "Robustness-Aware Tool Selection and Manipulation Planning with Learned Energy-Informed Guidance" (ICRA 2026): selects tools and plans trajectories with an "energy-based robustness metric". The abstract does not frame selection as mechanical advantage — [arXiv 2506.03362](https://arxiv.org/abs/2506.03362)
- Compliant object prying with a diffusion policy guided by vision and force (arXiv 2503.03998, battery disassembly): fixed-base; force feedback is "essential" for prying — [arXiv 2503.03998](https://arxiv.org/html/2503.03998v1)
- LocoMan's leg-mounted grippers and Helpful DoggyBot's mouth gripper are task-specific, pre-attached end-effectors on Go1/Go2. Neither reports forces (see section 2) — [arXiv 2403.18197](https://arxiv.org/html/2403.18197); [arXiv 2410.00231](https://arxiv.org/pdf/2410.00231)

### Inferences
- Holladay's "forceful kinematic chain" frames the student's crouch result directly: a force applied through the arm loads every joint in the chain. So crouching cannot add torque beyond what the shoulder and elbow can resist (about 1-1.5 N·m in the student's static model). A pre-attached tool helps only if it changes the load path or the lever arm, for example by bearing on the handle so the reaction goes into the base, or by lengthening the handle's effective radius. Holladay is the citation for treating tool choice as a constraint-satisfaction problem under joint torque limits.
- The ETH hook is the clearest legged precedent for a "pre-attached lever tool". Its 0-3 N·m simulated handle range brackets the student's combiner-box lever (0.4 N·m failing by grip; 1.15 N·m by arm push). Note that the ETH arm is far stronger than the D1, so the hook solves grip and slip, not torque. A D1 tool would need to solve torque as well.
- A "tool-conditioned policy" whose tool parameters (length, contact geometry) change the reachable force or torque set appears unpublished for legged robots. That is an inference from failing to find any, not proof of absence.

### Gaps
- I found no paper where a legged robot carries a lever, extension bar or wrench specifically to multiply a weak arm's torque.
- No learning paper conditions a policy on tool geometry to reach higher forces.
- The ETH door paper's arm model and torque were not stated in what I fetched; an earlier extraction said "specific arm model not detailed in excerpt".
- Hayakawa et al. report no masses in the abstract.

## 4. Bracing and environment contact: a second contact so the arm carries less load

### Takeaway
Bracing appears in three forms:
- **Environmental fixturing or support** of the object or the robot: Holladay's table, mat, vise and weight; Nakatsuru et al. using environment support to exceed payload; extrinsic dexterity in the style of Zhou & Held.
- **A second limb as a contact**: Sleiman et al. hold a spring-loaded door open with a leg while the arm works; Spot's front legs serve as optional manipulators in Sumo; the hexapod in Gong et al. leans on the block.
- **Posture planned to maximise the force the robot can resist**: RoLoMa raises the worst-case resistible force by up to about 18% through stance and posture alone.

None of these is on a low-cost arm, and none brace the arm itself against the environment to relieve its joints.

### Cited Findings
- Nakatsuru, Wan, Harada, "Implicit Contact-Rich Manipulation Planning for a Manipulator with Insufficient Payload" (arXiv 2302.13212, 2023): a mobile manipulator with a collaborative arm moves objects by "leveraging supporting forces from environmental obstacles". The abstract gives no numbers — [arXiv 2302.13212](https://arxiv.org/abs/2302.13212)
- Zhou & Held, "Learning to Grasp the Ungraspable with Emergent Extrinsic Dexterity" (CoRL 2022, oral): an RL policy learns to push the object against a wall to rotate it and then grasp it, with a simple gripper — [OpenReview](https://openreview.net/forum?id=xK-UtqDpD7L); [GitHub](https://github.com/Wenxuan-Zhou/ungraspable)
- Holladay et al. (IJRR): fixturing by table or mat friction, vise, second arm, or "weighing down the beam with a large mass" (see section 3) — [arXiv 2208.00319](https://arxiv.org/html/2208.00319)
- Sleiman, Farshidian, Hutter, "Versatile multicontact planning and control for legged loco-manipulation" (Science Robotics 2023): the planner discovers "what limbs it should use, as well as when and where it should establish or break contact". ANYmal opens heavy dishwashers and traverses spring-loaded doors, "keeping it open with a leg while maneuvering through". **No force numbers in the sources I read** — [arXiv 2308.09179](https://arxiv.org/html/2308.09179v1); [PubMed](https://pubmed.ncbi.nlm.nih.gov/37585544/); [TechXplore](https://techxplore.com/news/2023-08-closer-autonomous-robots.html)
- Sleiman, Mittal, Hutter, "Guided Reinforcement Learning for Robust Multi-Contact Loco-Manipulation" (CoRL 2024, oral): "navigating spring-loaded doors and manipulating heavy dishwashers"; learns recovery "such as re-grasping objects during execution or dealing with slippages". No force numbers in the abstract — [arXiv 2410.13817](https://arxiv.org/abs/2410.13817)
- RoLoMa (Autonomous Robots 2023), ANYmal B + Kinova:
  - Optimises the "smallest unrejectable force (SUF)", i.e. "the magnitude of the largest force that the robot can counteract in a worst-case scenario". The motivation is human pre-shaping: "leaning body backwards while pulling on a rope during a game of tug of war".
  - **Hand wheel turn:** SUF "73 N to 76 N" (proposed) vs "56 N to 66 N" (baseline).
  - **Lever pull:** "68 N to 75 N" vs "50 N to 69 N", about "18 %" mean increase.
  - **Rope-and-pulley bucket lift, stepping back while pulling:** succeeded at "1.1, 2.1, 3.1, and 4.1 kg". At 5.1 kg "the handle slipped off the gripper and we counted this as a failure".
  - [arXiv 2203.01446](https://arxiv.org/pdf/2203.01446)
- UniFP (arXiv 2505.20829) trains a policy on separate **base** force commands in [-60, 60] N alongside end-effector forces, so the policy is explicitly made to handle contact forces on the body — [arXiv 2505.20829](https://arxiv.org/html/2505.20829)
- Sumo (arXiv 2604.08508): "Legs function as stabilizing contact points during manipulation"; front legs optionally enabled for multi-limb manipulation — [arXiv 2604.08508](https://arxiv.org/html/2604.08508v1)
- Gong et al. review: "Leaning while holding establishes three points of" contact; leaning "provides stability ... due to the wider resulting base of support" — [arXiv 2303.16865](https://arxiv.org/pdf/2303.16865)

### Inferences
- RoLoMa's lever-pull failure mode at the load limit was the grip slipping, not joint torque running out. The student's pull fails the same way at 0.5 N·m. Both point to grip and contact, not torque, as the first limit for pull-type actuation with a small parallel gripper. A hook or pre-attached tool, or the tactile grip regulation shown by TAC-LOCO, addresses that limit.
- For the combiner box, the useful braces are probably (a) the robot's body or a foot against the cabinet, with the arm only positioning a tool, or (b) a tool shaped to hook or lock onto the lever so slip is removed. This is inference; I found no paper doing either on a lever with a small arm.

### Gaps
- No paper found braces a weak arm's links against the environment, as opposed to the object or the robot body, to relieve joint torque on a legged robot.
- I found no force numbers for Sleiman et al.'s spring-loaded doors in the sources fetched.

## 5. Dynamic or impulsive force: momentum, impacts, jerks

### Takeaway
I found **no paper** that deliberately uses body momentum, ramming or jerking to exceed a weak arm's static torque limit on a lever, door or stuck object. The evidence is indirect:
- RL in simulation discovers momentum exploits on door handles, which ETH explicitly suppressed (zero-friction hook contact).
- UMI on Legs shows emergent wiggling and force build-up until static friction breaks.
- Sumo calls its whole-body manipulation "dynamic".
- Impact-aware manipulation (EU I.AM.) uses intentional impacts on fixed-base arms, for speed rather than force.

### Cited Findings
- Zhang et al. (CoRL 2024) set the "friction coefficient of the handle and hook end-effector contact" to zero in simulation. Per the fetched summary, the purpose was to encourage learned behaviours rather than "aggressive momentum-based approaches". Momentum exploits on handles emerge in RL unless suppressed — [arXiv 2409.04882](https://arxiv.org/html/2409.04882v1)
- UMI on Legs (CoRL 2024): against static friction the controller keeps pushing, producing "a wiggling motion along with increased pushing force until static friction breaks into a dynamic slide". Tossing "requires dynamic whole-body coordination, involving power from all joints" — [arXiv 2407.10353](https://arxiv.org/html/2407.10353v1); [project](https://umi-on-legs.github.io/)
- Sumo / RAI Institute (arXiv 2604.08508, 2026): "dynamically manipulate large and heavy objects with whole-body dexterity"; 15 kg tire upright in an average 5.9 s against the arm's 11 kg peak lift — [RAI blog](https://rai-inst.com/resources/blog/combining-sampling-and-learning-for-dynamic-whole-body-manipulation/); [arXiv 2604.08508](https://arxiv.org/html/2604.08508v1)
- Gong et al. review: "Kicking imparts a quick impulse rather than a sustained force on the object"; quadruped kicking into a goal (Ji et al., 2022) used hierarchical RL — [arXiv 2303.16865](https://arxiv.org/pdf/2303.16865)
- I.AM. project (EU H2020): impact-aware dual-arm "swift grabbing" of heavy boxes. Reported a "29% decrease in average task time" and a "35% decrease" in energy against classical grabbing — [TUM portal](https://portal.fis.tum.de/en/publications/impact-aware-dual-arm-manipulation-a-framework-for-fast-package-h/); [I.AM. D5.4 report](https://i-am-project.eu/images/delivarables_public/IAM_-_D54_Scenario_2_BOX_report_reduced.pdf)
- Impact-Aware Task-Space QP Control (arXiv 2006.01987): control framework for intentional impacts on manipulators — [arXiv 2006.01987](https://arxiv.org/pdf/2006.01987)

### Inferences
- A spring-loaded lever is a near-static load: the spring torque rises with angle. An impulse can carry the handle past the latch release point only if the release angle is reached before the spring returns the handle. Whether that works depends on the handle's travel-to-release, which the student can test in simulation. It is a candidate strategy, but repeated impacts are a hardware risk for the D1's bus servos (see the overheating report in section 1).
- The ETH zero-friction trick is a useful precedent for a simulation-hygiene check: if a policy "succeeds" on the lever only through momentum or penetration artefacts, that should be reported as not validated.

### Gaps
- No paper found on ramming or jerking to open doors, levers or stuck objects with a legged robot, and no impulse force numbers for door handles.
- I did not find whether the ETH friction-zeroing was motivated by observed momentum exploits in their training runs; the phrase comes from a fetched summary of the paper.

## 6. Door, lever, valve and handle operation with weak or low-cost mobile manipulators: what worked, at what torque or force?

### Takeaway
Handle-level numbers are rare:
- **ETH door (CoRL 2024):** the one legged-robot paper that states a handle torque, simulating 0-3 N·m handle resistance and 0-30 N·m hinge resistance with a hook end-effector, at 95% real-world success.
- **RoLoMa:** a lever pull and a hand wheel with an ANYmal + Kinova, reported as worst-case resistible force (50-76 N), not handle torque.
- **Go2 + D1 tactile paper:** valve tightening without torque numbers.
- **Low-cost wheeled systems:** about $25k with a 5 kg xArm 6 (Xiong et al.); DoorBot with haptic feedback. Both report success rates, not handle torques.

For the student, the strategies that worked across all of these are: a hook or tool instead of a pinch grasp; a second contact (leg) to hold a spring-loaded door; posture or base motion to supply force; and haptic or force feedback to adapt.

### Cited Findings
- **ETH door, Zhang et al. (CoRL 2024), ANYmal + arm + hook:** handle resistance torque [0, 3] N·m; handle angle [15, 90]°; door mass [15, 75] kg; hinge [0, 30] N·m; real "95.0%" (20/20 pull, 18/20 push) — [arXiv 2409.04882](https://arxiv.org/html/2409.04882v1)
- **Pedipulate (ICRA 2024), ANYmal D, foot only:** pushed a door handle down with the foot (teleoperated), then walked through, pushing a spring-damper door; a fridge pull door opened by foot friction. No handle torque reported — [arXiv 2402.10837](https://arxiv.org/html/2402.10837)
- **RoLoMa (Autonomous Robots 2023), ANYmal B + Kinova:**
  - Lever pull: worst-case resistible force 68-75 N (proposed) vs 50-69 N (baseline).
  - Hand wheel, one full revolution: 73-76 N vs 56-66 N.
  - Industrial hand wheel turned multiple times in a repeatability test.
  - [arXiv 2203.01446](https://arxiv.org/pdf/2203.01446); [YouTube hand-wheel test](https://www.youtube.com/watch?v=Ok8Pcwn_I0w)
- **Go2 + D1 tactile paper (arXiv 2604.27224):** "Valve tightening with cyclic grasp-twist-release patterns"; no valve torque given — [arXiv 2604.27224](https://arxiv.org/html/2604.27224v3)
- **Sleiman et al. (Science Robotics 2023; CoRL 2024):** spring-loaded doors held open with a leg; heavy dishwashers — [arXiv 2308.09179](https://arxiv.org/html/2308.09179v1); [arXiv 2410.13817](https://arxiv.org/abs/2410.13817)
- **LocoMan (IROS 2024), Go1 + 147 g leg manipulators:** cabinet swing door and drawer; no forces — [arXiv 2403.18197](https://arxiv.org/html/2403.18197)
- **Rigo et al. (ICRA 2024), Aliengo + custom arm:** door manipulation alongside 8 kg payload carrying — [arXiv 2311.00112](https://arxiv.org/abs/2311.00112)
- **UniFP (B2 + Z1):** open-cabinet, close-cabinet and open-drawer tasks with force-aware control, about 39.5% above position-only baselines — [arXiv 2505.20829](https://arxiv.org/html/2505.20829)
- **Xiong, Mendonca, Shaw, Pathak, "Adaptive Mobile Manipulation for Articulated Objects in the Open World"** (arXiv 2401.14403, 2024; CVPR 2024 Embodied AI workshop): a "low-cost" platform at "around 25,000 USD", AgileX Ranger Mini 2 base + "xArm 6 for high-payload manipulation (up to 5kg)". 20 articulated objects across 4 CMU buildings; success rose "from 50% to 95%" with under an hour of online adaptation per object — [arXiv 2401.14403](https://arxiv.org/pdf/2401.14403); [project](https://door-open.github.io/)
- **DoorBot (Wang, Mo, Jin, Yuan; ICRA 2025):** closed-loop door opening with "real-time haptic feedback"; "90% success rate" over 20 unseen doors. The abstract gives no hardware or force numbers — [arXiv 2504.09358](https://arxiv.org/abs/2504.09358)
- **Video2DoorTraversal (arXiv 2608.20251, 2026, arXiv-only):** "coordinated base-arm control" for push doors; "96.57%" over five real doors and "80.95%" zero-shot on similar unseen doors. The abstract gives no robot or torque — [arXiv 2608.20251](https://arxiv.org/abs/2608.20251)
- **Spot Door Service:** AutoGrasp (grab handle, open, walk through); AutoPush (wrist push for push bars) — [Spot SDK](https://dev.bostondynamics.com/python/examples/arm_door/readme)

### Inferences
- The student's combiner-box lever (spring to about 0.4 N·m in the current sim; arm push reaching about 1.15 N·m) sits inside the 0-3 N·m handle range ETH trained over. So the lever is not unusually stiff compared with real door handles. The difficulty comes from the D1's 1.7-3.3 N·m joints and its pinch grip.
- On publishability (inference, for the supervisor's question):
  - The general claim that "whole-body control amplifies end-effector force beyond the arm's rating" is already published: Portela ICRA 2024 (90 N vs 36 N); UMI on Legs CoRL 2024 (leaning); Sumo 2026 (15-20 kg vs 11 kg); Jeon RA-L 2023 (body pushing).
  - Not found in the literature, and so a candidate contribution:
    1. A systematic, quantified comparison of force-bypass strategies on the same task for an arm whose joints, not the body, are the bottleneck: arm-only, crouch-with-arm, foot or body, pre-attached tool, brace, impulse.
    2. A tool-conditioned policy on a small quadruped.
    3. Lever and door operation on a sub-1 kg-payload, 10 Hz position-only arm such as the D1.
  - Each needs the scope caveat that simulation results are not hardware-validated.

### Gaps
- No paper I found reports the handle torque achieved by a low-cost arm (WidowX, ARX5, PiPER, D1) on a door lever or valve.
- I did not find industrial combiner-box or electrical-enclosure lever handle torque specifications.
- DoorBot's and Video2DoorTraversal's hardware was not in the abstracts fetched.
- The Pedipulate foot-force figure (12 N vs 130 N) conflicts between sources; see section 2.
