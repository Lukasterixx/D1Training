# Close matches on bracing and support-contact selection (scoop-risk check, read in full)

Read on 2026-10-01. Every fact below is tagged with where it came from:
**[FT]** = the paper's full text (PDF or arXiv HTML), read end to end; **[ABS]** = the abstract or landing page only;
**[SUPP]** = the paper's supplementary PDF; **[VENDOR]** = a product page; **[SEARCH]** = a search-engine summary only (not verified).
All four papers in the brief that exist were read in full. The fifth item in the brief (CTCS vs "Contact as a Decision Variable") turned out to be one paper, not two.

## Q0. Do the five papers exist, and what exactly does each do?

### Takeaway
Four distinct papers exist and all four were read in full. "CTCS" and "Contact as a Decision Variable" (Mahmud, Li & Wang) are the **same paper**: arXiv 2609.30140. The earlier "Brace Yourself" summary (≤13.5 N without bracing, up to 60 N with it) is correct but hides a confound. The 13.5 N hardware baseline is a different controller with an upright stance (the robot's onboard RL plus IK). The like-for-like "no-bracing" comparison was run only in simulation. The "paywalled" Scientific Reports paper is CC BY 4.0 open access. Its landing page shows only the abstract because it is an Article in Press, but the full unedited manuscript PDF can be downloaded.

### Cited Findings

#### 1. "Brace Yourself" (arXiv 2609.25486)
- **Identity [FT/ABS]:** "Brace Yourself: Task-Conditioned Environmental Bracing for Forceful Humanoid Manipulation". Authors: Zongyuan Zhang, Christopher Lehnert, Will N. Browne, Jonathan M. Roberts (QUT Centre for Robotics / Australian Cobotics Centre). arXiv v1 dated 21 Sep 2026. Comments: "Submitted to IEEE Robotics and Automation Letters (RA-L). 8 pages, 6 figures". Video at https://youtu.be/zPrGLxWJzI4, which I did not watch. — [arXiv abs](https://arxiv.org/abs/2609.25486); [PDF v1](https://arxiv.org/pdf/2609.25486v1)
- **Platform [FT]:** a 29-DoF Unitree G1 humanoid. One hand braces against a planar surface (the "Supporting Hand Strategy", SHS) while the other hand does a forceful task on a rigid vertical surface. The paper gives **no arm torque or payload figures**. It only says the shoulder-pitch actuator "was consistently the first joint on either arm to approach its thermal limit". — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Method [FT]:** the method has two parts:
  - **Offline optimisation (TRSO).** SLSQP finds a single support configuration (foot-midpoint position, ankle pitch, pelvis pitch, waist and supporting-arm joints) that works across a task region of 30 samples at 0–75 N. Each sample still gets its own working-arm joints and supporting-hand normal force. The cost covers hand pose error, residual wrench at the foot midpoint, friction and ZMP inside the foot polygon, plus joint regularisation. The result is encoded as 18-D keypoints (WBKR).
  - **Two synchronous PPO policies.** A body policy (legs, waist, supporting arm; 22 actions) and a working-arm policy (7 actions) output joint-position targets for joint PD control. Policies run at 50 Hz and the joint controller at 200 Hz. **Actors get no measured force**; force is used only by the critics and rewards. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **How torque limits enter [FT]:** only as a reward penalty. Torque utilisation is penalised as the 4th-power norm of |τ_j|/τ_j^max over each arm's joints, ramped in by a curriculum, to manage actuator heating. TRSO has no working-arm torque constraint: its box constraints are joint limits and foot-position bounds. The support configuration is chosen for **balance (ZMP/friction)**, not to offload the working arm's joints. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Contact location [FT]:** the second hand (the supporting arm) against an environmental planar surface. The working hand still carries the full task force through its own arm joints. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Measurement [FT]:** an ATI Axia80-M20 six-axis F/T sensor at the working wrist. Hand and shoulder positions come from joint-encoder forward kinematics. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Hardware protocol [FT]:**
  - **Experiment 1:** the target normal force is stepped from 0 to 80 N in 10 N levels at 5 fixed positions. Each level is tested 3 times and counts as "usable" if it is sustained in ≥2 of the 3 tests.
  - **Experiment 2:** an 8 s figure-eight at 10/25/40/55/70 N. Generic vs SHS recordings per level: 6/6, 6/7, 6/5, 6/7, 5/7.
  - Modular was recorded on 29 trajectories.
  - Transfer regions T2 and T3 (shifted 5 cm laterally and vertically) were run at 10–55 N. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Baselines and ablations [FT]:**
  - **Generic:** task-independent medoid reference.
  - **No-reference:** no WBKR guidance.
  - **No-bracing:** bracing reward removed, with a task-matched forward lean kept. Simulation only.
  - **Modular (hardware baseline):** the robot's onboard lower-body RL controller, IK for the working arm, and a PI loop on measured normal force. No supporting-hand contact, upright stance. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Results, hardware [FT]:**
  - **Usable force:** SHS reached 60 N at the centre and upper positions and 40 N at the lower positions. Generic reached 40–50 N. Modular was "usable only at 10 N at the lower-right position and never sustained more than 13.5 N for one second".
  - **Maximum 1-second sustained force (Fig. 4a):** read from the figure labels in the PDF text. SHS ≈ 58.5–72.4 N, Generic ≈ 49.2–64.6 N, Modular ≈ 12.6–13.5 N. Which value belongs to which position is my reading of the figure layout.
  - **Tracking at 55 N:** SHS cut force RMS error from 13.1 to 9.2 N and tangential RMS error from 5.1 to 3.6 cm.
  - **Modular tracking:** Modular tracked only 11 of 29 recordings, all at ≤11 N. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Results, simulation (30 seeds per method) [FT]:**
  - Bracing raised episode success from 0.020 to 0.666 and stable-contact hold ratio from 0.069 to 0.950 compared with No-bracing.
  - Only 2 of the 30 No-bracing policies kept contact, and both "produced > 21 N for an 8 N target and retained load at zero target force". The paper's reading: "the forward lean therefore routes body load through the working hand, imposing a nonzero minimum force". — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **The authors' explanation of the cap [FT]:** "Modular keeps an upright stance and cannot maintain the forward lean required to balance large reaction forces". The cap is framed as **balance and reaction-force resistance**, not as working-arm joint torque. — [PDF](https://arxiv.org/pdf/2609.25486v1)
- **Stated limitations [FT]:**
  - The whole task region must be reachable from one support configuration.
  - Only one planar support surface was tested.
  - Other contact planes, curved surfaces, and supporting-hand grasps or hooks (which would give tensile support) are untested. — [PDF](https://arxiv.org/pdf/2609.25486v1)

#### 2 and 3. CTCS = "Contact as a Decision Variable" (arXiv 2609.30140): one paper, not two
- **Identity [FT/ABS]:** "Contact as a Decision Variable: Capability-Tradeoff Contact Selection for Legged Loco-Manipulation". Authors: Al Jaber Mahmud, Shuai Li, Xuan Wang (George Mason University; Li at the University of Florida). arXiv v1 dated 24 Sep 2026. "9 pages, 6 figures", no venue stated. — [arXiv abs](https://arxiv.org/abs/2609.30140); [PDF v1](https://arxiv.org/pdf/2609.30140v1)
- **Platform [FT]:** a Unitree Go2 with an **AgileX NERO 7-DoF arm** and "a rigid planar support frame mounted on the side of its body for establishing environmental contact". This is the "body-mounted brace frame" mentioned in the brief. — [PDF](https://arxiv.org/pdf/2609.30140v1)
- **Arm spec [VENDOR]:** NERO payload 3 kg, reach 580 mm, arm mass 4.8 kg, 0.1 mm repeatability, CAN/TCP/HTTP interface, about €3,599. The vendor page lists no joint torques or control modes. The paper itself gives no arm torque numbers. — [OpenELAB NERO page](https://openelab.io/products/agilex-nero-embodied-ai-arm)
- **Problem [FT]:** jointly choose (a) which of 9 environmental surfaces to contact with the body frame and (b) the whole-body configuration, for a prescribed end-effector wrench, end-effector position and base velocity.
  - The surfaces are 1 wall, 4 tables and 4 horizontal rails, each 5 m long.
  - Two modes: "sticking" (4-foot stance, static friction) and "walking" (trot phases, contact slides along the surface with kinetic friction).
  - The objective is a weighted sum of three "residual capabilities" minus a contact-acquisition cost:
    - residual wrench: the radius of the largest ball inside the 6-D residual-wrench polytope after the task wrench is met;
    - residual end-effector reach;
    - residual base mobility. — [PDF](https://arxiv.org/pdf/2609.30140v1)
- **How torque limits enter [FT/SUPP]:** as hard constraints |u_i| ≤ u_max,i on all 19 actuated joints (12 leg + 7 arm). They sit inside the per-direction feasibility LPs, alongside friction cones, ground normal-force bounds (λ̄g = 400 N, μg = 0.6) and a support normal-force bound (λ̄c = 250 N). **The u_max values are not given.** The paper does not report which constraint (arm torque, leg torque, friction or rollover) is the binding one. — [PDF](https://arxiv.org/pdf/2609.30140v1); [Supplementary](https://supplementary-material1.pages.dev/supplementary.pdf)
- **Candidate sampling [FT]:** "the sampling bounds keep the legs away from kinematic singularities, including full extension, and the manipulator away from singular configurations". Near-singular postures are therefore **deliberately excluded**. — [PDF](https://arxiv.org/pdf/2609.30140v1)
- **Method type [FT]:** optimisation and selection, not RL.
  - **Selection pipeline:** 5000 sampled candidates per surface (45,000 in total). Exact feasibility screening, then complete-linkage grouping, anchor LP solves, and first-order sensitivity prediction via the implicit function theorem. A probe test and a cross-anchor check validate the predictions, then the top-ranked shortlist is evaluated exactly.
  - **Execution:** a receding-horizon whole-body controller running at **10 Hz** on hardware. — [PDF](https://arxiv.org/pdf/2609.30140v1); [Supplementary](https://supplementary-material1.pages.dev/supplementary.pdf)
- **Simulation results (Gazebo; 392 conditions = 2 modes × 14 force directions × 14 magnitudes from 5 to 70 N) [FT]:**
  - **Certification rate:** CTCS 91.8% vs ground-only 68.4% (sticking) and 65.3% (walking). Fixed-contact reached only 48.5% and 41.8%.
  - **Mean objective:** 3.5419 vs ground-only 2.5324 (sticking); 5.6612 vs 3.9184 (walking).
  - **Speed:** 3.17× (sticking) and 2.53× (walking) faster than exhaustive search, with 95.6% and 100% selection agreement.
  - **Ablations:** CTCS-NV, CTCS-SA and CTCS-FFS all lowered the objective. — [PDF](https://arxiv.org/pdf/2609.30140v1)
- **Hardware results [FT]:** five trials each for one sticking task and one walking task. The experimenter applies a load at the end effector, and Fig. 1(e) shows a "Force Gauge Reading". The text also refers to a "measured wrist-reaction force" without naming the sensor. In the representative walking trial the peak applied force was 25 N, at 0.1 m/s base speed.
  - Sticking: force RMSE 0.229 N; end-effector position RMSE 6.48 mm.
  - Walking: force RMSE 0.278 N; mean base-velocity error −0.00547 m/s; end-effector position RMSE 14.97 mm.
  - "Without environmental support, due to the weight of the arm, the robot will roll over." This is a qualitative claim; **no hardware run without support is reported**. — [PDF](https://arxiv.org/pdf/2609.30140v1)
- **Stated limitations [FT]:** selection is over sampled candidates, so it is not globally optimal. Future work: candidate refinement, contact transitions, and online reselection. — [PDF](https://arxiv.org/pdf/2609.30140v1)

#### 4. Scientific Reports 2026: Šifrer & Petrič
- **Identity [FT]:** "Enabling extended tool manipulation through redundancy-driven contact exploitation". Authors: Jan Šifrer and Tadej Petrič (Jožef Stefan Institute, Ljubljana). *Scientific Reports* (2026), DOI 10.1038/s41598-026-60582-y. Received 12 May 2026, accepted 29 June 2026, online 3 July 2026. It is an **Article in Press**, released as an "unedited version", and licensed CC BY 4.0. The full manuscript is the `_reference.pdf` linked from the article page. — [Article page](https://www.nature.com/articles/s41598-026-60582-y); [Unedited manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Platform [FT]:** Franka Research 3 (7-DoF, torque-controlled) run in **joint-velocity mode at 500 Hz**. A 1.4 m rigid stick with a paint roller is attached near one end, on purpose, to maximise the lever-arm effect. An **ATI Nano25 F/T sensor at the flange** feeds both the force control and the null-space objective. — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Method [FT]:**
  - The primary task is tool-tip position plus normal-force admittance control.
  - The secondary task, in the null space, drives the measured flange force toward zero, which pushes the arm to find support.
  - Three variants: Moore–Penrose pseudoinverse, damped least squares, and a directional formulation.
  - A passive adjustable support **beneath the stick**, neither modelled nor sensed, becomes a contact by emergence. In the student's vocabulary this is a fulcrum on the tool, giving a parallel load path through the environment. — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Explanation given [FT]:** the paper explains the mechanism in load-path terms. A passive support contact "can in principle provide an additional load path through the environment". Redundancy is "a mechanism for modifying force-transmission pathways under the lever-arm effect of long tools". — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Experiments [FT]:**
  - Fixed endpoint, with the support removed at t = 15 s; Fz rises to a higher steady value.
  - Support repositioned in height and horizontally.
  - Sinusoidal tracking.
  - Normal-force regulation at a commanded **8 N**.
  - Combined painting motion.
  - A local wall obstruction.
  - Results are shown only as force-trace plots. The text reports **no numeric joint-torque reduction, no trial counts and no statistics**. The authors call it "a mechanism-level demonstration … rather than … a full statistical robustness study". — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Stated limitations [FT]:**
  - Moderate force only. Static moments at the sensor were "on the order of several Nm", which is comparable to the Nano25 range.
  - No explicit wrench or joint-torque limit constraints.
  - No friction-cone or slip handling, and μ is not estimated.
  - Tangential force is not regulated.
  - Speed limits were not determined.
  - "Should not be interpreted as validation for high-force industrial processes". — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Minor inconsistency:** the funding line differs between the web page (ARIS programme P2-0076) and the manuscript (ARIS grant N2-0269). — [Article page](https://www.nature.com/articles/s41598-026-60582-y); [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)

#### 5. Fang et al. 2019 (the "[15]" in Brace Yourself; also "[1]" in CTCS)
- **Identity [FT]:** C. Fang, N. Kashiri, G. F. Rigano, A. Ajoudani, N. G. Tsagarakis, "Exploitation of Environment Support Contacts for Manipulation Effort Reduction of a Robot Arm". ICRA 2019, pp. 9502–9508, DOI 10.1109/ICRA.2019.8794119 (IIT; Fang later at SDU). The accepted-manuscript copy is in SDU's repository. — [Accepted manuscript (SDU)](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf); [DOI](https://doi.org/10.1109/ICRA.2019.8794119)
- **Platform and task [FT]:** the 7-DoF right arm of the IIT **Centauro** robot, fixed base. The **elbow** lands on a horizontal support plane. A 2.5 kg payload sits at the end effector, and only the first 4 joints (3-DoF shoulder plus elbow) are used. The task is to hold a posture and run a "conceptual pick-and-place". — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)
- **Method [FT]:**
  - Offline: plan the landing point, then pick a landing posture that minimises apparent inertia at the contact point, to limit impact.
  - Online: a three-level passivity-based hierarchical compliance controller (after Dietrich et al.).
    - Level 1: impedance normal to the plane, switching smoothly to an "optimal contact force" that minimises ‖τ‖₂.
    - Level 2: tangential impedance at the contact point.
    - Level 3: end-effector impedance.
  - No F/T sensor is needed at the contact. — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)
- **Results [FT]:**
  - **Table II, six landing postures:** max impact force 310.9–592.4 N. Joint-torque-norm reduction 60.8%–87.1% compared with holding the same posture without contact (posture a: 76.2%; f: 87.1%). Footnote 3 says the landing distance and speed were increased "in simulation" for this study, so Table II appears to be simulation.
  - **Pick-and-place simulation:** contact force up to 340 N after impact; torque norm up to 15 N·m during landing.
  - **Hardware:** shown only as a with-contact vs without-contact joint-torque plot (Fig. 6) with "similar experimental results". The text gives no hardware numbers or trial counts and does not say how hardware joint torques were measured. — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)
- **Assumptions and limitations [FT]:** the contact point is assumed stable (sliding is ignored). Manipulation is commanded quasi-statically. The contact uses up DoFs ("the elbow is a suitable part for support contact leaving 5 DoFs for the task execution"). — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)

### Inferences
- Of the five papers, only CTCS uses a quadruped with an arm, and it uses the Go2, the student's own base. That makes CTCS the closest platform match. Its contact, however, is **body ↔ environment** and supports the base against rollover and ground limits. The end-effector wrench still passes through all seven NERO joints in series.
- The paper whose mechanism is closest to "a parallel path around weak joints" is Fang 2019: elbow contact unloads the shoulder joints, a non-end-effector link contact on the arm itself. Šifrer & Petrič come next, with tool-on-support contact that redistributes the tool's lever-arm load. Both are fixed-base arms.

### Gaps
- None of the papers gives arm joint-torque limits in N·m: the G1 arm in Brace Yourself, the NERO in CTCS (u_max not listed), and Centauro in Fang 2019.
- CTCS does not say how the "wrist-reaction force" was measured on hardware (Fig. 1(e) shows only a force gauge), what loads were applied in the sticking trials, or which constraints were active.
- I did not watch the Brace Yourself video. I did not check whether a CTCS project page or code exists; neither appears in the arXiv metadata.

## Q1. Does any of these papers already show, on a quadruped with an arm, that body posture alone is capped by the arm's joint torques and that parallel contacts or bracing lift the cap?

### Takeaway
**No.** None of the four shows that cap on a quadruped, or on any legged robot. CTCS (Go2 + NERO) shows that a body-mounted support contact increases the set of feasible task wrenches compared with ground-only support. The evidence is simulation certification rates, and the cap is never attributed to arm joint torques. Brace Yourself (G1 humanoid) shows bracing lifts usable force from ≤13.5 N to 40–60 N on hardware, but it blames **balance**, not arm torque, for the unbraced cap. Its braced working arm still carries the full force in series.

### Cited Findings
- **CTCS: what it measures.** It compares "ground-only" support with environmental support in Gazebo by **certification rate** (does any sampled configuration satisfy the task?): 68.4% → 91.8% (sticking) and 65.3% → 91.8% (walking). It does not compare arm-only against body-posture strategies, and it reports no force-capability ratio. — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1)
- **CTCS: what is constrained.** Torque limits for all 19 joints are hard constraints, alongside friction, ground-force and support-force bounds. The paper never says which one binds, and the support frame is on the body, not the arm. — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1); [Supplementary](https://supplementary-material1.pages.dev/supplementary.pdf)
- **CTCS: the no-support case on hardware.** The only statement is the qualitative "Without environmental support, due to the weight of the arm, the robot will roll over". That is a balance or tipping limit, not an arm-torque limit. — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1)
- **Brace Yourself: the authors' explanation.** "Modular keeps an upright stance and cannot maintain the forward lean required to balance large reaction forces". Separately, "without a second contact the lean load must pass through the working hand, imposing a non-zero minimum force". — [Brace Yourself PDF](https://arxiv.org/pdf/2609.25486v1)
- **Brace Yourself: the confound.** The 13.5 N hardware baseline (Modular: onboard RL + IK + PI force loop, upright) differs from SHS in controller, stance and bracing all at once. The clean bracing-only ablation (No-bracing) exists only in simulation. — [Brace Yourself PDF](https://arxiv.org/pdf/2609.25486v1)
- **Brace Yourself: arm torque.** It enters only as a thermal-motivated torque-utilisation **penalty** in the RL reward. Bracing is optimised for ZMP and friction, not for arm-joint unloading. — [Brace Yourself PDF](https://arxiv.org/pdf/2609.25486v1)
- **Fixed-base papers.** Fang 2019 quantifies the arm-joint relief from a parallel elbow contact: a 60.8–87.1% reduction in torque norm, apparently in simulation. Šifrer & Petrič describe the relief qualitatively as an "additional load path through the environment". Neither involves a legged base or body posture. — [Fang 2019 manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf); [Šifrer & Petrič manuscript](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)

### Inferences
- The student's specific claims are still open:
  - On a quadruped, re-posing the body with a rigid arm gains only ~1.3× because the reaction passes through the shoulder and elbow in series.
  - Parallel paths (bracing, body or leg contact, a fulcrumed tool, strut postures) are what lift the cap.
- The two September 2026 preprints sit right next to this, and a reviewer will cite them:
  - CTCS: same base robot, body-mounted brace, explicit torque limits, a residual-wrench capability metric.
  - Brace Yourself: a hardware before/after force cap from bracing.
- The student should therefore set their contribution against both explicitly:
  - (a) Name the binding constraint (arm joint torque, not balance or rollover) and show it with numbers.
  - (b) Run arm-only vs body-posture vs parallel-path on one task.
  - (c) Point out that CTCS's brace is in parallel with the legs, not with the arm.
- Brace Yourself's No-bracing observation is the closest published analogue to the student's "crouch with a rigid arm" case: a forward lean without a second contact routes body load through the working hand. That observation is simulation-only, on a humanoid, and framed as a force-regulation problem (a forced minimum force), not as a torque cap. It is worth citing as a consistent but differently explained result.

### Gaps
- CTCS does not report whether arm torque limits were active in the selected candidates, so I cannot say whether its residual-wrench gains come from arm-torque relief or from balance and friction relief.
- I found no hardware arm-only vs bracing comparison on a quadruped in any of the four papers.

## Q2. Does any of them operate a lever, handle, valve or door?

### Takeaway
**No.** None of the four operates a lever, handle, valve or door, and none reports a task torque about a hinge or pivot in N·m. The tasks are pushing normal force on a wall (Brace Yourself), sustaining an applied end-effector load (CTCS), wall painting with a long tool (Šifrer & Petrič), and holding a payload during pick-and-place (Fang 2019).

### Cited Findings
- **Brace Yourself:** the working hand holds or tracks figure-eights on a rigid vertical surface at a target normal force of 10–80 N, measured with a wrist F/T sensor. — [Brace Yourself PDF](https://arxiv.org/pdf/2609.25486v1)
- **CTCS:**
  - **Simulation tasks** are specified only as a required end-effector wrench: 14 directions × 5–70 N. Plus an end-effector position and, for walking, a 0.1 m/s base velocity.
  - **Hardware:** an experimenter applies a downward load in sticking mode, and a load of up to 25 N while walking.
  - The motivating example is a water cannon. — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1); [Supplementary](https://supplementary-material1.pages.dev/supplementary.pdf)
- **Šifrer & Petrič:** wall-painting with a 1.4 m roller stick at 8 N normal force. The only N·m figure is the static moment at the flange sensor, "on the order of several Nm". — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Fang 2019:** holding a posture and doing a conceptual pick-and-place with a 2.5 kg payload. The only N·m figure is the joint-torque norm, up to 15 N·m in simulation. — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)

### Inferences
- Lever or door actuation against a spring, with the result reported as torque about the pivot in N·m, is unclaimed by these four papers. So is the related idea of using the pivoting object's own geometry, for example a fulcrum or the box body as a brace.

### Gaps
- This check covers only these four papers. Other September 2026 preprints on legged door or lever opening were outside this brief and were not searched.

## Q3. Are any of them on a low-cost, position-controlled arm?

### Takeaway
**No arm here is comparable to the D1, a few-N·m servo arm with position-only commands at about 10 Hz.** The weakest is the NERO (3 kg payload, €3.6k). Brace Yourself does send joint-position targets, but to the G1's own PD-controlled actuators at 50/200 Hz, with forces of 60 N. The other two use a torque-controlled FR3 and the high-strength Centauro arm.

### Cited Findings
- **Brace Yourself:** the policies output "joint-position targets as a per-joint scaled offset from the default posture q0, tracked by joint proportional–derivative control". Policies run at 50 Hz and the joint controller at 200 Hz, on a 29-DoF Unitree G1. — [Brace Yourself PDF](https://arxiv.org/pdf/2609.25486v1)
- **CTCS:** the arm is an AgileX NERO 7-DoF. The receding-horizon whole-body controller optimises joint torques u and runs at 10 Hz on hardware. The paper does not say what command interface the NERO and Go2 joints receive. — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1); [Supplementary](https://supplementary-material1.pages.dev/supplementary.pdf)
- **NERO spec:** 3 kg payload, 4.8 kg arm mass, 580 mm reach, about €3,599. No joint torques are listed. — [OpenELAB NERO page](https://openelab.io/products/agilex-nero-embodied-ai-arm)
- **Šifrer & Petrič:** "Franka Research 3 (FR3), a 7-DOF torque-controlled manipulator operating in joint-velocity control mode at … 500 Hz". The method needs a flange F/T sensor (ATI Nano25). The authors present the problem as relevant to "low- to medium-payload collaborative manipulators". — [Manuscript PDF](https://www.nature.com/articles/s41598-026-60582-y_reference.pdf)
- **Fang 2019:** Centauro arm, described in its reference [34] as a "human size and strength compliant bi-manual platform for realistic heavy manipulation tasks". The controller is torque-level (passivity-based impedance). — [Accepted manuscript](https://findresearcher.sdu.dk/ws/portalfiles/portal/157251188/ICRA2019_Cheng_Fang.pdf)

### Inferences
- **Hardware and controller.** None of the four has the student's combination: a low-cost servo arm with no joint-torque sensing and no F/T sensor, commanded by position only at about 10 Hz, whose joint torque is the binding limit.
  - Šifrer & Petrič and Fang both rely on torque-level control or F/T feedback to find and keep their support.
  - Brace Yourself's actors use no force feedback, which partly parallels a position-only arm. But the G1's actuators are far stronger than the D1's.
- **What remains open, across all four papers:**
  1. Arm-only vs body-posture vs parallel-path comparison on the same task, on a quadruped: **open** (CTCS compares ground-only vs contact in simulation by feasibility, not by force cap).
  2. Explicit series-vs-parallel load-path explanation of the posture cap: **partly anticipated in wording only.** Šifrer & Petrič write "additional load path"; Fang describes contact relieving the non-task joints. Neither applies it to a legged body, and none quantifies a ~1.3× posture-only cap.
  3. Near-singular "strut" postures: **open.** CTCS explicitly samples the manipulator away from singular configurations. Šifrer & Petrič use damped least squares to stay robust near singularities. Nobody uses singularity as a deliberate force path.
  4. A weak position-controlled servo arm: **open.**
  5. Lever or door torque in N·m: **open.**

### Gaps
- None of the papers gives arm joint-torque limits, so I cannot compare their arms' torque headroom with the D1's in N·m.
- Two adjacent prior works turned up in CTCS's related work and were not read in full.
  - Wolfslag et al., IROS 2020, "Optimisation of Body-ground Contact for Augmenting the Whole-Body Loco-manipulation of Quadruped Robots". From the search summary only: prongs on the quadruped body create body-ground contact, with an HQP controller and a "Smallest Unrejectable Force" metric. — [Edinburgh Research Explorer](https://www.research.ed.ac.uk/en/publications/optimisation-of-body-ground-contact-for-augmenting-whole-body-loc) [SEARCH]
  - Natarajan et al., ICRA 2023, "Torque-limited manipulation planning through contact…". CTCS describes it as using bracing to "enable torque-limited manipulation tasks, and enlarge the workspace reachable under torque limits". — [CTCS PDF](https://arxiv.org/pdf/2609.30140v1); [CMU PDF](http://biorobotics.ri.cmu.edu/papers/paperUploads/85259-2703.pdf) (not read)
  - Wolfslag in particular, a quadruped using body contact to augment manipulation, should be read in full before novelty claims about "body or leg contact" are made.
