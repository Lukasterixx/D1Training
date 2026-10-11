# Prior work: legged robots using body, legs, posture or weight to exert manipulation forces beyond the arm's own capability

Scope notes for the report writer: the search covered arXiv, PMLR, IEEE/RA-L listings, project pages and vendor specs, about 45 tool calls, on 2026-10-01. Numbers are quoted as the sources state them. "arXiv-only" means I found no peer-reviewed venue. Where a fetch returned only an abstract, missing numbers are listed under Gaps and not estimated.

## Q1. Which papers have a legged base (quadruped, with or without an arm) supply manipulation force and quantify it? Which only coordinate base and arm for reach?

### Takeaway
Only a few quadruped papers both use the body for force and give numbers against the arm's rating. The clearest are Portela et al. (ICRA 2024: a 90 N pull against the Z1 arm's 36 N rated payload) and the RAI Institute/Sumo Spot work (15–20 kg objects against the Spot arm's 11 kg peak lift). Neither isolates the mechanism or compares arm-only with whole-body force. Most other "whole-body" quadruped papers, such as Deep WBC, VBC and RoboDuet, coordinate base and arm for reach and workspace, not force. The body-only pushing papers (Jeon 2023, Feng 2025) show the body producing force with no arm at all.

### Cited Findings

**A. Quadruped + arm, body/legs used for force, with numbers**

- **Portela, Margolis, Ji, Agrawal, "Learning Force Control for Legged Manipulation," ICRA 2024 (also ICRA24 loco-manipulation workshop), arXiv 2405.01402.**
  - Platform: Unitree B1 quadruped (55 kg, 0.64 m tall, 12 actuators) with a Unitree Z1 arm (6-DoF, 5.3 kg, 0.74 m reach, "rated payload of 36 N").
  - Method: RL (PPO), sim-to-real, no force/torque sensor. Force commands span 0–70 N. Tracking error was 5–10 N across five dynamometer trials; in simulation the error is about 5 N for low targets and below 10 N across the training range.
  - Verbatim: "The observed force of 90 N is greater than the arm's rated payload of 36 N." The measurement setup: "we initialized the arm directly in front of the robot and recorded the highest pulling force we can achieve."
  - The paper says the system can "coordinate the body with the legs to increase the applied force". Whole-body control enlarged the workspace volume by 59% over arm-only control.
  - It gives no arm-only maximum-force comparison and no mechanism analysis. Arm torque appears only as a reward penalty when τa > 0.8 × |τa^max|.
  - Sources: [arXiv abs](https://arxiv.org/abs/2405.01402); [arXiv HTML](https://arxiv.org/html/2405.01402v1).
- **Zhi et al., UniFP: "Learning a Unified Policy for Position and Force Control in Legged Loco-Manipulation," CoRL 2025 (PMLR v305), arXiv 2505.20829.**
  - Platforms: Unitree B2 with a Z1 arm; a 29-DoF Unitree G1 humanoid.
  - Method: RL with no force sensors. Training samples end-effector force commands in [-60 N, 60 N] and, notably, a separate base force command in [-60 N, 60 N]. The policy turns force errors into position offsets at the end-effector and velocity offsets at the base.
  - Real-world results: force-control error "within 10 N"; force-estimation error 5–10 N. The Y and Z axes were evaluated only to 40 N "due to hardware constraints".
  - Force-aware imitation learning had about 39.5% higher success than vision-only (e.g. open-cabinet 0.70 vs 0.36).
  - It does not frame the result as exceeding the arm's capability.
  - Sources: [arXiv HTML](https://arxiv.org/html/2505.20829); [PMLR](https://proceedings.mlr.press/v305/zhi25a.html); [GitHub](https://github.com/unified-force/UniFP).
- **Sleiman, Farshidian, Hutter, "Versatile multicontact planning and control for legged loco-manipulation," Science Robotics 8(81), 16 Aug 2023, arXiv 2308.09179.**
  - Platform: ANYmal with a "custom-built 6-DoF robotic arm".
  - Method: model-based. Task and motion planning with trajectory optimisation, graph search and sampling, then a two-layer whole-body tracking controller.
  - Tasks: a heavy dishwasher door with "large manipulation forces of up to 125 N due to the high friction in the dishwasher's joint", and spring-loaded push, pull and sliding doors.
  - Legs act as non-prehensile manipulators: feet "hold the door as the arm either switches to a new object contact", and one foot is used to "lift the dishwasher" while closing it. This is a force path that bypasses the arm.
  - Sources: [arXiv abs](https://arxiv.org/abs/2308.09179); [arXiv HTML](https://arxiv.org/html/2308.09179); [Science Robotics](https://www.science.org/doi/10.1126/scirobotics.adg5014).
- **Sleiman, Farshidian, Minniti, Hutter, "A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation," RA-L 6:4688–4695, 2021 (presented at ICRA 2021), arXiv 2103.00946.**
  - Method: model-based MPC. The centroidal dynamics are augmented with the manipulated object's dynamics.
  - Hardware: "pushing/pulling a heavy resistive door".
  - The fetched abstract gives no force or mass numbers and does not name the arm. One search summary claimed the robot leans "its entire body into a heavy door"; that is not verified in the abstract.
  - Source: [arXiv](https://arxiv.org/abs/2103.00946).
- **Zhang, Ma, Miki, Hutter, "Learning to Open and Traverse Doors with a Legged Manipulator," CoRL 2024 (PMLR v270, 2025), arXiv 2409.04882.**
  - The task brief named Rudin and Hoeller as authors; PMLR lists Mike Zhang, Yuntao Ma, Takahiro Miki and Marco Hutter.
  - Platform: ANYmal with an arm. The extracted text does not name the arm model.
  - Method: RL teacher-student. A single policy infers push or pull. Arm actions are clipped by torque limits with saturation parameter σ = 0.7.
  - Simulation randomisation: door mass 15–75 kg, hinge resistance 0–30 Nm, handle resistance 0–3 Nm.
  - Results: 95.0% real-world success over 20 trials per door side. In simulation, passing success is "near zero" for hinge resistance above 50 Nm.
  - The base contributes through velocity commands (max 0.5 m/s, 1 rad/s). The paper does not quantify force supplied by the body.
  - Sources: [arXiv HTML](https://arxiv.org/html/2409.04882v1); [PMLR](https://proceedings.mlr.press/v270/zhang25g.html).
- **Bellicoso et al., ALMA, ICRA 2019.**
  - Platform: torque-controlled ANYmal with a Kinova Jaco2 6-DoF arm (4.4 kg, torque control on all six actuators).
  - Method: model-based (whole-body control plus ZMP locomotion).
  - It opens a spring-loaded door by "commanding a contact force task instead of a motion task", and carries a payload together with a human.
  - Torso posture optimisation serves to "increase the arm's workspace", not force.
  - The PDF was unreachable (404), so I have no force numbers.
  - Sources: [Semantic Scholar](https://www.semanticscholar.org/paper/ALMA-Articulated-Locomotion-and-Manipulation-for-a-Bellicoso-Kr%C3%A4mer/e599f77ac0f28a9dc2141fd0eef00aef0c8ade22); [bibbase](https://bibbase.org/network/publication/bellicoso-kramer-stuble-sako-jenelten-bjelonic-hutter-almaarticulatedlocomotionandmanipulationforatorquecontrollablerobot-2019).
- **RAI Institute, Spot tire manipulation (blog/video, no paper linked).**
  - Spot uprights, rolls, drags and stacks a "15 kg" car tire, "well above its peak lift capacity of 11 kg and continuous lifting capacity of 5 kg for an object carried in the gripper".
  - Quote: "a tight coupling between locomotion and manipulation expands the operational range beyond conventional pick-and-place systems".
  - Method: RL (PPO) for locomotion and rolling, plus sampling-based MPC (32 CPU threads of MuJoCo rollouts) for uprighting, dragging and stacking.
  - It uses "multi-limb, multi-contact behavior" across arm, legs and body.
  - Source: [RAI blog](https://rai-inst.com/resources/blog/combining-sampling-and-learning-for-dynamic-whole-body-manipulation/).
- **Zhang, Sorokin, Brüdigam, Hung et al., "Sumo: Dynamic and Generalizable Whole-Body Loco-Manipulation," arXiv 2604.08508 (April 2026; arXiv-only as found).**
  - Platform: Spot with arm (real); Unitree G1 (simulation).
  - Method: a pre-trained RL whole-body policy steered by real-time sampling-based MPC.
  - Real tasks: tire upright 15 kg, crowd barrier upright 16 kg, chair upright 16.5 kg, tire stack 15 kg, rugged box push 20 kg.
  - It states explicitly that the tire "is heavier than the maximum lifting capacity of 11kg of the Spot arm". Quote: "using a combination of its arm, torso, and legs, Sumo enables the robot to complete the task 10 out of 10 trials".
  - This is the most explicit "exceeds the arm's rating" framing I found for a quadruped.
  - Source: [arXiv HTML](https://arxiv.org/html/2604.08508v1).
- **Boston Dynamics Spot Arm specification (vendor).**
  - Lift "Up to 11 kg"; continuous lift "5 kg" at 0.5 m extension; drag "Up to 25 kg (on carpet)"; gripper peak clamp force "130 N"; all at 22 °C.
  - The vendor's own drag rating is more than twice its lift rating.
  - Source: [Spot SDK arm spec](https://dev.bostondynamics.com/docs/concepts/arm/arm_specification.html).
- **Rigo, Hu, Gupta, Nguyen, "Hierarchical Optimization-based Control for Whole-body Loco-manipulation of Heavy Objects," arXiv 2311.00112 (venue not confirmed in this pass).**
  - Platform: Unitree Aliengo with a custom arm.
  - Method: model-based (manipulation planning, pose optimisation, linear MPC that includes the manipulation forces).
  - It lifts and carries "an 8kg payload and manipulate[s] doors", and shows "the necessity of whole-body optimization compared to the baseline locomotion MPC".
  - Source: [arXiv](https://arxiv.org/abs/2311.00112).
- **Dadiotis, Laurenzi, Tsagarakis, "Trajectory Optimization for Quadruped Mobile Manipulators that Carry Heavy Payload," Humanoids 2022, arXiv 2210.06803.**
  - Platform: CENTAURO (bimanual quadruped).
  - Payload "exceeds 15 % of its mass". The TO accounts for payload dynamics and gives "reduced leg outstretching".
  - Source: [arXiv](https://arxiv.org/abs/2210.06803).
- **Schperberg, Wang, Di Cairano (MERL), "Safe Whole-Body Loco-Manipulation via Combined Model and Learning-based Control," arXiv 2603.02443 (March 2026; arXiv-only).**
  - Platform: Unitree Go2 with a D1 6-DoF servo arm and a BOTA wrist F/T sensor. This is the student's base and arm.
  - Control: arm admittance at 40 Hz ("D1 arm motor send/receive frequency is limited at 40 Hz"); RL locomotion at 50 Hz.
  - The base is "a slower, high-inertia, long-range body that repositions to sustain the motion".
  - Tasks: human-robot plank co-transport and force-following.
  - It reports velocity-tracking MSE (≤0.005 m²/s² linear) but no force-capacity numbers, and it does not discuss D1 torque limits.
  - Source: [arXiv HTML](https://arxiv.org/html/2603.02443).
- **Mahmud, Li, Wang, "Contact as a Decision Variable: Capability-Tradeoff Contact Selection for Legged Loco-Manipulation," arXiv 2609.30140 (24 Sep 2026; arXiv-only).**
  - Platform: Unitree Go2 with an AgileX NERO arm.
  - It selects an environmental support contact (out of nine surfaces) jointly with the whole-body configuration, trading off "residual wrench, end-effector reach, and base mobility".
  - Evaluated over 392 task conditions; about 3× speedup while matching the exhaustive search's objective.
  - The fetched abstract gives no force numbers.
  - Source: [arXiv](https://arxiv.org/abs/2609.30140).
- **Zeng et al., "Force-Aware Reinforcement Learning with Hybrid Sensorless Force Estimation for Wheeled-Legged Loco-Manipulation," arXiv 2609.13779 (arXiv-only).**
  - Platform: wheeled-legged robot with an arm.
  - Method: momentum observer plus residual learning for force estimation.
  - The abstract gives no force numbers.
  - Source: [arXiv](https://arxiv.org/abs/2609.13779).
- **Das, Byrd, Baek, Ha, "Partial Motion Imitation for Learning Cart Pushing with Legged Manipulators," arXiv 2603.26659 (arXiv-only).**
  - Simulation only (IsaacLab and MuJoCo). Lower-body motion is imitated with partial adversarial motion priors.
  - The abstract gives no cart masses or forces.
  - Source: [arXiv](https://arxiv.org/abs/2603.26659).

**B. Quadruped body/legs as the manipulator (no arm, or leg instead of arm)**

- **Jeon, Jung, Choi, Kim, Hwangbo, "Learning Whole-body Manipulation for Quadrupedal Robot," IEEE RA-L 2023 (DOI 10.1109/LRA.2023.3335777), arXiv 2308.16820.**
  - Platform: a 27 kg quadruped with no arm.
  - Method: hierarchical RL with a latent embedding of object properties.
  - Real-world: a 19.2 kg water-filled drum and a 15.3 kg loaded box. Simulation: 93.6% success within 0.03 m and 5°.
  - Sources: [arXiv](https://arxiv.org/abs/2308.16820); [Semantic Scholar](https://www.semanticscholar.org/paper/Learning-Whole-Body-Manipulation-for-Quadrupedal-Jeon-Jung/435b06453230184b0b23e32a8e65e93895f92c80).
- **Feng, Hong, Niu, Liu, Yang, Yu, Zhang, Tan, Zhao, "Learning Multi-Agent Loco-Manipulation for Long-Horizon Quadrupedal Pushing," arXiv 2411.07104 v4 (29 Mar 2025; venue not confirmed).**
  - Platform: Unitree Go1 robots, each with "an approximate payload capacity of 5 kg", in teams of 2–4.
  - Objects: cuboid 6.8 kg real (4 kg sim); T-block 3.3 kg real; cylinder 10 kg sim.
  - Quote: "a single robot has limited strength". Single-agent success was below 25%, against above 60% for their method.
  - It gives no force numbers.
  - Source: [arXiv HTML](https://arxiv.org/html/2411.07104v4).
- **Arm, Mittal, Kolvenbach, Hutter, "Pedipulate," ICRA 2024.**
  - A leg of ANYmal is used as the manipulator, trained with RL. It carries "more than 2.0 kg at the foot".
  - Source: [ETH Research Collection](https://www.research-collection.ethz.ch/handle/20.500.11850/660204) (summary via search).
- **HyQReal (IIT), debut at ICRA 2019 (demo, not a force study).**
  - A 130 kg hydraulic quadruped pulled a "3.3 ton" Piaggio P180 airplane for more than 10 m.
  - Sources: [EurekAlert](https://www.eurekalert.org/news-releases/713173); [IIT DLS](https://dls.iit.it/hyqreal).
- **Multi-quadruped towing of a cable-towed load.**
  - A search result listed "Decentralized Navigation of a Cable-Towed Load using Quadrupedal Robot Team via MARL" (arXiv 2503.18221) together with a citation to RA-L vol. 7(4), pp. 10041–10048 (2022).
  - The RA-L citation probably belongs to an earlier cable-towing paper; the search snippet appears to merge the two. Not verified.
  - Source: [arXiv PDF](https://arxiv.org/pdf/2503.18221).

**C. Base–arm coordination for reach and workspace (not force): the contrast class**

- **Fu, Cheng, Pathak, "Deep Whole-Body Control," CoRL 2022.**
  - Platform: Unitree Go1 with a "lightweight 6-DoF WidowX 250 S arm".
  - A unified RL policy for end-effector tracking and locomotion; no force objective.
  - Sources: [OpenReview](https://openreview.net/forum?id=zldI4UpuG7v); [GitHub](https://github.com/MarkFzp/Deep-Whole-Body-Control).
- **VBC, "Visual Whole-Body Control for Legged Loco-Manipulation," arXiv 2403.16967 (OpenReview-listed; venue not re-verified).**
  - Platform: Unitree B1 with a Unitree Z1 arm.
  - Low-level whole-body tracking with high-level visual planning; pickup of 14 objects at three heights.
  - Sources: [arXiv HTML](https://arxiv.org/html/2403.16967v5); [OpenReview](https://openreview.net/forum?id=cT2N3p1AcE).
- **RoboDuet, arXiv 2403.17367 (venue not verified).**
  - Platform: Unitree Go1 Edu with a 6-DoF ARX5 arm.
  - Two cooperative policies for large-range 6D pose tracking.
  - Source: [arXiv](https://arxiv.org/pdf/2403.17367).
- **ALMA (above)**: torso posture optimisation is for "increas[ing] the arm's workspace" ([Semantic Scholar](https://www.semanticscholar.org/paper/ALMA-Articulated-Locomotion-and-Manipulation-for-a-Bellicoso-Kr%C3%A4mer/e599f77ac0f28a9dc2141fd0eef00aef0c8ade22)).
- **Portela 2024**: the 59% workspace enlargement is a reach result, reported separately from the 90 N force result ([arXiv HTML](https://arxiv.org/html/2405.01402)).

### Inferences
- **Portela's "90 N > 36 N" is the closest quadruped precedent** for "the body amplifies a weak arm". It compares a pulling force measured with the arm in front of the robot against a rated lift payload, not against an arm-only pull in the same pose, so the amplification factor attributable to the body is not isolated.
  - The student's arm-only vs crouch analysis (1.15 N·m vs median 0.94 / best 1.5 N·m, about 1.3×) is exactly the controlled comparison that Portela lacks.
- **The Spot results (RAI blog, Sumo) do bypass the arm's lift limit**, but they mostly do it with non-prehensile contacts. Tires and barriers are tipped, rolled or dragged with torso and legs, so much of the load never passes through the arm's joints. This supports the student's hypothesis that large gains need force paths that bypass the arm's motors.
- **Spot's vendor rating of drag (25 kg) at more than twice lift (11 kg)** suggests that ground-reaction-supported pulling is less arm-limited than lifting. The spec does not explain the mechanism.
- **Sleiman 2023 is the clearest model-based precedent for legs as auxiliary force paths** (feet holding or lifting a door while the arm regrasps), but it is framed as versatility, not as overcoming arm weakness.

### Gaps
- No force or torque numbers were retrieved for ALMA (PDF 404), Sleiman 2021 (abstract only; arm model unnamed), Rigo 2023 (forces not in abstract), CTCS 2609.30140, Zeng 2609.13779 or Das 2603.26659.
- Venues are unconfirmed for Rigo (2311.00112), Feng (2411.07104), VBC and RoboDuet.
- Zhang et al. 2024 does not state the arm model or arm torque limits in the text I could extract.
- I did not find a published Boston Dynamics paper quantifying Spot door-opening forces. Spot door opening appears only in product material, not checked here.
- Not checked: "Legs as Manipulator" (Cheng, Kumar, Pathak, ICRA 2023), UMI on Legs (arXiv 2407.10353), Schwarke et al. (CoRL 2023, ANYmal + DynaArm door), OpenHEART (arXiv 2603.05830) and the Go2 calf-integrated arms paper (arXiv 2607.06186). They appeared in searches but were not fetched.

## Q2. Humanoid analogues: leaning or using weight to push or pull heavy loads

### Takeaway
The humanoid literature is ahead of quadrupeds in framing body posture as a force multiplier.
- Thor (G1) derives a load-dependent torso tilt and centre-of-mass offset from quasi-static moment balance and shows it raises peak pull (167.7 N; towing a 1.7 t car).
- FALCON bounds the trained forces by the joint-torque limits.
- "Brace Yourself" (Sept 2026) adds a second, bracing hand and gets a 4.4–5.9× increase in sustainable force.
- Classic model-based work (Murooka 2015 on HRP-2; Polverini 2020 on CENTAURO) plans whole-body pushing postures for heavy objects.

### Cited Findings
- **Li, Shi, Shi, Hu, Ma, Wang, Wang, Luo, "Thor: Towards Human-Inspired [v1: Human-Level] Whole-Body Reactions for Intense Contact-Rich Environments," arXiv 2510.26280 (v1 30 Oct 2025, v4 21 Sep 2026; no venue listed).**
  - Platform: Unitree G1.
  - Method: RL with three jointly trained policies (lower body, waist, upper body). The FAT2 objective "derives a load-dependent horizontal center-of-mass offset reference from quasi-static moment balance".
  - Mean peak dual-hand pulling force is 167.7 N backward and 145.5 N forward, "exceeding the best-performing baseline by 68.9% and 74.7%". Real-world ablations show that "FAT2 increases peak pulling capability".
  - Demonstrations: one-hand fire-door opening at "approximately 60 N" and "towing a 1.7-ton passenger car". An earlier abstract version, quoted in a search summary, also listed "pulling a loaded rack (130 N)".
  - Sources: [arXiv abs](https://arxiv.org/abs/2510.26280); [alphaXiv v1](https://alphaxiv.org/overview/2510.26280v1).
- **Zhang, Yuan, …, He (CMU LeCAR), "FALCON: Learning Force-Adaptive Humanoid Loco-Manipulation," arXiv 2505.06776; L4DC 2026 (oral) per the GitHub README.**
  - Platforms: Unitree G1 and Booster T1.
  - Method: dual-agent RL with a "torque-limit-aware 3D force curriculum". Feasible end-effector force per axis comes from inverse dynamics, fᵢᵐᵃˣ = minⱼ((τⱼˡⁱᵐ − τⱼᵍ)/|J_EE,ji| + ε).
  - The paper notes humanoids with "relatively weak joint torque limits, such as the wrist joints".
  - Forces: payload transport 0–20 N vertical; cart pulling "up to 100N"; door opening 0–40 N. Wider test ranges: X/Y [-100, 100] N, Z [-100, 5] N.
  - The lower body "leans against significant longitudinal forces".
  - Tracking error under large forces is 0.37 rad, against 0.60 (PID) and 0.73 (monolithic RL). Methods that ignore torque limits "saturated at force scale αg = 0.6".
  - Sources: [arXiv HTML](https://arxiv.org/html/2505.06776); [GitHub](https://github.com/LeCAR-Lab/FALCON); [project](https://lecar-lab.github.io/falcon-humanoid/).
- **Zhang, Lehnert, Browne, Roberts (QUT), "Brace Yourself: Task-Conditioned Environmental Bracing for Forceful Humanoid Manipulation," arXiv 2609.25486 (21 Sep 2026; arXiv-only).**
  - Platform: Unitree G1 (29 DoF) with an Axia80 wrist F/T sensor.
  - Method: an RL "Supporting Hand Strategy" in which one hand braces on the environment while the other works.
  - Results: "usable forces up to 60 N" at centre and upper positions and 40 N at lower positions. The no-bracing modular baseline "never sustained more than 13.5 N for one second", a 4.4–5.9× increase.
  - Tracking at 55 N: normal-force RMS error 13.1 → 9.2 N; shoulder wobble reduced 2.7–3.8×.
  - Shoulder-pitch thermal limits are handled with a fourth-power penalty on normalised joint torque.
  - Sources: [arXiv HTML](https://arxiv.org/html/2609.25486v1); [project](https://brace-yourself-robotics.github.io/).
- **Hu, Xu, Chang, Yin, Tran, Martín-Martín, Stone, Qiao, Biswas, "VOFA: Visual Object Goal Pushing with Force-Adaptive Control for Humanoids," IROS 2026 (accepted), arXiv 2605.01518.**
  - Platform: Booster T1.
  - It pushes objects up to 17 kg, "exceeding half the Booster T1 humanoid's body weight".
  - Success is over 90% in simulation and over 80% in the real world.
  - Source: [arXiv](https://arxiv.org/abs/2605.01518).
- **Xiao, Xu, Zhe, Liu, Chen, "Kinematics-Aware Multi-Policy Reinforcement Learning for Force-Capable Humanoid Loco-Manipulation," arXiv 2511.21169 (arXiv-only).**
  - Method: a three-stage RL pipeline whose lower body uses a "force-based curriculum ... to actively exert and regulate interaction forces".
  - It includes cart pushing and pulling with payloads; no numbers were retrieved.
  - Source: [arXiv](https://arxiv.org/abs/2511.21169).
- **Dong et al., "HAFO: A Force-Adaptive Control Framework for Humanoid Robots in Intense Interaction Environments," arXiv 2511.20275 (arXiv-only).**
  - Method: dual-agent RL with spring-damper disturbance modelling, covering load-bearing, thrust and rope suspension.
  - No numbers were retrieved.
  - Source: [arXiv](https://arxiv.org/abs/2511.20275).
- **Zheng et al., "Embracing Bulky Objects with Humanoid Robots: Whole-Body Manipulation with Reinforcement Learning," arXiv 2509.13534 (arXiv-only).**
  - Whole-body multi-contact embracing gives "stable multi-contact interactions that enhance the robustness in manipulation and also the load capacity".
  - No numbers were retrieved.
  - Source: [arXiv](https://arxiv.org/abs/2509.13534).
- **Murooka, Nozawa, Kakiuchi, Okada, Inaba, "Whole-body pushing manipulation with contact posture planning of large and heavy object for humanoid robot," ICRA 2015, pp. 5682–5689.**
  - Platform: HRP-2. Method: model-based.
  - It pre-computes "several different pushing postures that exert increasing amounts of force", escalates when the object does not move, and adapts footsteps.
  - Object masses were not available in the sources I reached.
  - Sources: [ResearchGate](https://www.researchgate.net/publication/283768584_Whole-body_pushing_manipulation_with_contact_posture_planning_of_large_and_heavy_object_for_humanoid_robot); [IEEE Spectrum](https://spectrum.ieee.org/robots-learn-to-push-heavy-objects-with-their-bodies-just-like-you).
- **Polverini, Laurenzi, Hoffman, Ruscelli, Tsagarakis, "Multi-Contact Heavy Object Pushing With a Centaur-Type Humanoid Robot: Planning and Control for a Real Demonstrator," RA-L 5(2):859–866, 2020.**
  - Platform: CENTAURO.
  - Method: model-based. Nonlinear optimisation over centroidal statics, plus hierarchical IK and reactive contact-force distribution.
  - It pushes "a wooden cabinet loaded with bricks"; the mass was not available.
  - Source: [ADS abstract](https://ui.adsabs.harvard.edu/abs/2020IRAL....5..859P/abstract).
- **Rouxel, Ivaldi, Mouret, "Multi-Contact Whole-Body Force Control for Position-Controlled Robots" (SEIKO), RA-L 9(6):5639–5646, 2024.**
  - Platform: Talos humanoid.
  - It controls contact forces indirectly on position-controlled joints using an explicit flexibility model. Extra contacts such as "placing a hand on a wall or a handrail" are used in pushing and far-reaching tasks.
  - Relevant because the D1 is a position-controlled servo arm.
  - Source: [arXiv](https://arxiv.org/abs/2312.16465).

### Inferences
- **Thor's FAT2 is the humanoid version of "lean into the load".** It shifts the centre of mass so that body weight balances the pulling moment at the feet. The pulling force still passes through the arms; it is the balance limit, not the arm torque limit, that FAT2 raises.
  - This suggests a two-limit picture for the student's setup: (i) tip-over/balance limits, where body weight and posture help directly, and (ii) arm joint-torque limits, where posture helps only by reshaping the arm's Jacobian.
  - The student's 1.3× gain is consistent with being arm-limited, not balance-limited.
- **"Brace Yourself" is the closest conceptual match**: a second contact chain shares the load, and the gain is quantified (4.4–5.9×). It is on a humanoid and published in Sept 2026, so the area is active.

### Gaps
- I could not retrieve Thor's full text (HTML 404 for v1 and v4). Whether Thor discusses arm/shoulder torque saturation as a limit is unknown.
- Murooka 2015 and Polverini 2020 masses and forces were not retrieved.
- Harada et al.'s ZMP-based humanoid pushing (early 2000s, before the 2015 window) and Ohmura & Kuniyoshi's whole-body-contact lifting were not fetched.
- The humanoid door paper "Opening the Sim-to-Real Door for Humanoid Pixel-to-Action Policy Transfer" (arXiv 2512.01061) appeared in searches; its force numbers were not checked.

## Q3. Does any paper analyse why body weight cannot help when the force still passes through the arm's joints, and propose ways around it?

### Takeaway
I found no paper that states and quantifies the student's specific limitation: crouching or putting weight onto an arm-held contact cannot exceed the arm's joint-torque limits, so the gain stays small (about 1.3×). Several papers formalise pieces of it:
- FALCON derives feasible end-effector force from the arm's joint-torque limits.
- FORTE optimises base pose to reshape a residual force polytope under the arm's torque limits.
- CTCS (Go2) chooses support contacts by residual wrench.
- Bracing papers ("Brace Yourself"; Book 1985; Fang 2019) add contact chains to cut arm joint effort.

The workarounds that appear in the literature are bracing, leg or body contacts that bypass the arm (Sleiman 2023 feet; Jeon 2023 body pushing; RAI/Sumo torso-and-leg contacts), base or posture optimisation for force capability (FORTE), and torque-limit-aware training (FALCON).

### Cited Findings
- **FALCON (arXiv 2505.06776; L4DC 2026)** computes the maximum admissible end-effector force per axis from joint torque limits minus gravity torque, divided by the Jacobian entry: fᵢᵐᵃˣ = minⱼ((τⱼˡⁱᵐ − τⱼᵍ)/|J_EE,ji| + ε). It notes that "it is crucial to explicitly account for these torque constraints when large external disturbances are applied to the end-effectors". This is the student's bottleneck written as a constraint, though FALCON uses it for curriculum design, not to argue for alternative force paths. [arXiv HTML](https://arxiv.org/html/2505.06776)
- **Wang, Zhang, Solak, Zhao, Ajoudani, "FORTE: Task-Adaptive Force Capability Optimization for Mobile Manipulators," Robotics and Autonomous Systems (journal, per the arXiv page), arXiv 2609.21497.**
  - Platform: a wheeled base with a Franka 7-DoF arm, τ_max "60.0 Nm (i=1,…,4), 12.0 Nm (i=5,…,7)".
  - Metric: the signed distance between a task-force uncertainty ball and the dynamic residual force polytope.
  - It optimises the whole-body configuration, including base pose, to "reshape the robot's end-effector capability while preserving the desired end-effector trajectory".
  - Results are qualitative: sufficient force capability for heavy loads while keeping manipulability for light ones.
  - It is not legged.
  - Source: [arXiv HTML](https://arxiv.org/html/2609.21497).
- **Mahmud, Li, Wang, CTCS, arXiv 2609.30140 (Go2 + AgileX NERO).** Support-contact selection jointly with whole-body configuration, scored on "residual wrench, end-effector reach, and base mobility". [arXiv](https://arxiv.org/abs/2609.30140)
- **"Brace Yourself" (arXiv 2609.25486)** states the mechanism: "interaction forces applied by the hand can propagate through the kinematic chain, disturb whole-body balance, and displace the floating base, ultimately limiting the force that the robot can sustain". Bracing with the other hand raised sustained force from ≤13.5 N to 40–60 N.
  - Its related work says environmental bracing has been used "to reduce joint effort, enable torque-limited manipulation tasks, and enlarge the workspace reachable under torque limits". It cites:
    - Book et al., "The Bracing Strategy for Robot Operation" (RoManSy '84, 1985);
    - Fang et al., "Exploitation of Environment Support Contacts for Manipulation Effort Reduction of a Robot Arm" (ICRA 2019);
    - Wang & Minami (SICE 2010).
  - Source: [arXiv HTML](https://arxiv.org/html/2609.25486v1).
- **Sleiman et al. 2023 (Science Robotics)** uses feet as non-prehensile end-effectors that hold or lift doors, so part of the manipulation load bypasses the arm. [arXiv HTML](https://arxiv.org/html/2308.09179)
- **Sumo (arXiv 2604.08508) and the RAI blog** handle objects heavier than the Spot arm's 11 kg lift limit "using a combination of its arm, torso, and legs". [Sumo](https://arxiv.org/html/2604.08508v1); [RAI](https://rai-inst.com/resources/blog/combining-sampling-and-learning-for-dynamic-whole-body-manipulation/)
- **Portela 2024** handles arm torque only as a reward penalty (τa > 0.8 × |τa^max|) and does not analyse how the body raises force. [arXiv HTML](https://arxiv.org/html/2405.01402v1)
- **Zhang et al. 2024 (doors)** clips arm actions by torque limits (σ = 0.7) but does not analyse the force path. [arXiv HTML](https://arxiv.org/html/2409.04882v1)
- **Rouxel et al. 2024 (SEIKO)** adds hand-on-wall or handrail contacts for position-controlled robots during pushing tasks. [arXiv](https://arxiv.org/abs/2312.16465)

### Inferences
- **The student's finding can be stated as a known principle applied to a new setting.** With the arm rigid, joint torques are τ = Jᵀ F at the contact. Body weight and base posture change F's direction relative to the arm and the arm's own gravity load, but they cannot raise any |τⱼ| above its limit.
  - So the arm-path gain is bounded by how much the posture can align F with the arm's "strong" directions; the student measured about 1.3×.
  - Larger gains need force paths with fewer or no arm joints in series: a leg or body contact (Jeon, Sleiman 2023, Sumo), bracing a second link (Brace Yourself), or near-singular, fully stretched postures in which the load is carried axially by the links.
  - The singular-posture route is standard force-manipulability reasoning, not something these sources demonstrate for legged robots.
- **For the D1 specifically**, the binding base-yaw joint on a lever push suggests placing the body so the line of force passes close to the base-yaw axis. That is a FORTE-style base-pose optimisation for force capability, untested on legged robots in the sources found.
- **Publication angle.** I found no paper that, on a weak low-cost arm on a small quadruped:
  - (a) measures arm-only versus body-posture force capacity for the same task (Portela's 90 N lacks this baseline);
  - (b) shows that posture gains are capped by arm joint torques;
  - (c) compares force paths that bypass the arm (leg or body contact, bracing, singular arm configurations) on a spring-loaded lever or door with N·m results.

  The closest neighbours are both from September 2026 and arXiv-only: Brace Yourself (humanoid bracing, 4.4–5.9×) and CTCS (Go2 support-contact selection by residual wrench). The topic is active, so the novelty case would rest on the weak-arm framing, the controlled comparison and the industrial-task benchmark, and it should be checked against these two papers before any claim is made.

### Gaps
- Book et al. 1985 and Fang et al. 2019 are cited through "Brace Yourself" only; I did not read them.
- I found no paper that reports a quantitative "posture gain" ratio for force through a fixed arm on a quadruped, so this is a negative finding from a limited search, not proof of absence.
- Google Scholar "cited by" chains could not be traversed with the available tools. Coverage of 2025–2026 follow-ups to Portela, ALMA and Sleiman relies on keyword search.

## Q4. Which works use a low-cost or weak arm, and which use industrial-grade arms where arm torque is not the bottleneck?

### Takeaway
Low-cost or weak arms on small quadrupeds dominate the learning papers: WidowX 250 S, ARX5, Unitree Z1, D1 and NERO. Only Portela reports a force exceeding such an arm's rating, and none analyses the arm as the force bottleneck. The model-based force papers mostly use torque-controlled or industrial-class arms (ANYmal with Kinova Jaco2 or a custom 6-DoF arm, Spot Arm, CENTAURO, Franka). The one Go2 + D1 paper (MERL 2026) studies safe admittance, not force capacity.

### Cited Findings
- **Low-cost or weak arms on small or medium quadrupeds:**
  - Go1 + WidowX 250 S (Deep WBC, CoRL 2022) — [GitHub](https://github.com/MarkFzp/Deep-Whole-Body-Control)
  - Go1 Edu + ARX5 (RoboDuet) — [arXiv](https://arxiv.org/pdf/2403.17367)
  - B1 + Z1 (VBC) — [arXiv](https://arxiv.org/html/2403.16967v5)
  - B1 + Z1, "rated payload of 36 N" (Portela, ICRA 2024) — [arXiv HTML](https://arxiv.org/html/2405.01402v1)
  - B2 + Z1 (UniFP, CoRL 2025) — [arXiv HTML](https://arxiv.org/html/2505.20829)
  - Go2 + D1 6-DoF servo arm with a 40 Hz comms limit (Schperberg et al., arXiv 2603.02443) — [arXiv HTML](https://arxiv.org/html/2603.02443)
  - Go2 + AgileX NERO (CTCS, arXiv 2609.30140) — [arXiv](https://arxiv.org/abs/2609.30140)
  - Go2 + Airbot Play (FT-WBC, arXiv 2606.24466; listed in search, not fetched) — [arXiv](https://arxiv.org/pdf/2606.24466)
- **Torque-controlled or industrial-class arms:**
  - ANYmal + Kinova Jaco2 (4.4 kg, torque control on all six joints; ALMA, ICRA 2019) — [Semantic Scholar](https://www.semanticscholar.org/paper/ALMA-Articulated-Locomotion-and-Manipulation-for-a-Bellicoso-Kr%C3%A4mer/e599f77ac0f28a9dc2141fd0eef00aef0c8ade22)
  - ANYmal + custom 6-DoF arm, 125 N dishwasher forces (Sleiman 2023) — [arXiv HTML](https://arxiv.org/html/2308.09179)
  - ANYmal + arm, hinge resistance up to 30 Nm in training (Zhang 2024) — [arXiv HTML](https://arxiv.org/html/2409.04882v1)
  - Spot Arm: 11 kg lift, 5 kg continuous at 0.5 m, 25 kg drag — [BD spec](https://dev.bostondynamics.com/docs/concepts/arm/arm_specification.html)
  - CENTAURO (Dadiotis 2022; Polverini 2020) — [arXiv](https://arxiv.org/abs/2210.06803); [ADS](https://ui.adsabs.harvard.edu/abs/2020IRAL....5..859P/abstract)
  - Franka, 60 / 12 Nm limits (FORTE) — [arXiv HTML](https://arxiv.org/html/2609.21497)
  - Aliengo + custom arm, 8 kg payload (Rigo) — [arXiv](https://arxiv.org/abs/2311.00112)
- **No-arm baselines (body as the manipulator):**
  - 27 kg quadruped pushing 19.2 kg and 15.3 kg objects (Jeon, RA-L 2023) — [arXiv](https://arxiv.org/abs/2308.16820)
  - Go1 teams, about 5 kg payload each, pushing up to 6.8 kg real (Feng 2025) — [arXiv HTML](https://arxiv.org/html/2411.07104v4)
- **Humanoid arms:** FALCON notes that humanoids have "relatively weak joint torque limits, such as the wrist joints" (G1, Booster T1) — [arXiv HTML](https://arxiv.org/html/2505.06776). "Brace Yourself" manages G1 shoulder-pitch thermal limits with a torque-utilisation penalty — [arXiv HTML](https://arxiv.org/html/2609.25486v1).
- **Supporting context:** the Go2 + D1 combination is also used in a Caltech SURF integration report — [PDF](https://eloisezeng.people.caltech.edu/documents/32338/2025_SURF_Final_Report.pdf) (not read; title only).

### Inferences
- **The weak-arm, force-limited regime is under-studied.** The papers that report the largest manipulation forces use torque-controlled or industrial arms, where arm torque is less often the binding limit. The low-cost-arm papers mostly target reach or tracking. Portela is the exception: a force above the arm's rating, but without mechanism analysis.
- **A Go2 + D1 study of force capacity would sit in a gap** between Portela (B1 + Z1, a 55 kg base with a much stronger arm) and the MERL Go2 + D1 paper (compliance and safety, not force).

### Gaps
- Published joint-torque limits for the WidowX 250 S, ARX5, NERO and the DynaArm were not collected here. Only the Z1's "rated payload of 36 N" (as stated by Portela) and the Franka limits (as stated by FORTE) are sourced.
- D1 torque specs come from the student's brief, not re-verified.
