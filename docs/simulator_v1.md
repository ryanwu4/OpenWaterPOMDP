# Simulator v1 spec

Scope: the true world dynamics, i.e. `dynamics.jl`, `drafting.jl`, `field.jl` and `gen` in `pomdp.jl`. Observations and beliefs are out of scope here.

Decisions are tagged **[decided]** (chosen in the design interview) or **[default]** (filled in to make the spec complete; open for discussion).

---

## 1. World

- **Course** [decided]: buoy at (L, 0) with L = 1500 m. Swimmers start at x = 0, with y spread over ±20 m (narrowed from ±100 m so drafting actually occurs).
- **Start** [decided]: all swimmers, the agent included, start at t = 0 at a random y. Initial heading points at the buoy, plus a small Gaussian aiming error.
- **Current** [decided]: one constant vector c = (cx, cy) per race, the same everywhere on the course and fixed for the race. It is drawn from a prior in the params.
- **No contact** [decided]: swimmers don't collide or block each other and can overlap in space. They interact only through drafting.

## 2. Time and units

- **Decision cycle** [decided]: one breathing cycle of fixed stroke count. A cycle without a sight lasts τ₀ seconds. A sight adds τκ seconds, so Δt = τ₀ + τκ·m.
- **Sync** [decided]: every step advances the whole world by the agent's Δt. Field swimmers move for that same duration.
- **Continuous-time core** [decided]: the simulator is built around one function, `advance(world, T, rng)`, which moves every swimmer forward by T seconds. `gen` is simply `advance` by the agent's Δt, wrapped with the agent's action and the reward. Inside `advance`, each swimmer's own events happen at their actual times, not at the end of the agent's step (see §4.1).
- **Continuous-time rates** [default, forced by sync]: field swimmers' own cycles don't line up with the agent's Δt, so every dynamic quantity is a per-second rate scaled by actual swim time:
  - pace v in m/s, so forward distance = v · t_swim
  - heading bias b in rad/s, so heading drift = b · t_swim
  - heading noise: variance σψ² · t_swim
  - position noise: variance σw² · Δt per axis
  - field sight schedule: N_k strokes is converted into a sight period in seconds

  The noise terms are Brownian, so splitting an interval into segments and adding up independent per-segment noise gives the same distribution as one draw over the whole interval. That is what makes the segmented `advance` exact.
- **Episode length**: at 1500 m and about 1.3 m/s, a race takes about 1150 s, or about 460 decisions at τ₀ = 2.5 s. Planners will need rollouts or a value estimate to see the finish. τ₀ is a parameter in case a coarser cycle is needed.

## 3. Swimmer motion (solo swimmers)

Heading drift [decided] is a random walk plus a per-swimmer bias, i.e. a fixed pull to one side drawn per race.

A swimmer is always in one of two motion states:

- **Swimming**: velocity v·(cos ψ, sin ψ) + c; the heading drifts with bias and noise.
- **Pausing to sight** for τκ: velocity c only (no forward swimming, no heading drift).

Both states also get position noise. Over a segment of length h:
- position += (velocity)·h + w, with w ~ N(0, σw²·h·I)
- if swimming, ψ ← ψ + b·h + ε, with ε ~ N(0, σψ²·h)

Over a segment, the displacement uses the heading at the start of the segment, and drift is applied at its end. This is an Euler step; segments are short (§4.1), so the error is small.

The agent's step [default] is a fixed schedule of its own events:

1. At t₀, clamp the turn δ to [−δmax, δmax] and apply it: ψ ← ψ + δ. Turning is continuous and bounded, with no cost [decided].
2. Swim for τ₀.
3. If m = 1, pause for τκ.

The step ends at t₀ + Δt. The sight timing [decided] is "sight now, turn next": the observation (built elsewhere) is taken at the end position s′, so information from a sight can only affect the next step's δ.

Field swimmers follow the same motion. Their sight pauses and re-aims are events at their own times (§5), and those times can fall anywhere inside the agent's step.

## 4. Drafting

### Geometry
- **Zone** [decided]: behind the leader only. In the leader's heading frame the follower must be 0 ≤ g ≤ g_max behind the leader and within ±w_lat laterally. g is the along-track gap.
- **Eligibility** [default]: the agent's follow/solo choice is checked once, on the state at the start of the step, s. Field swimmers' draft contacts are checked at every segment boundary inside `advance` (§4.1).

### Who can follow whom
- **One follower per leader** [decided].
- **Chains** [decided]: banned, except that the agent may be a middle link (field → agent → field leader). The agent's leader must itself be solo, because a field swimmer can't be a middle link.
- **Field follows field / field follows agent / agent follows field** [decided]: all allowed.
- **Contested spot** [decided]: if two swimmers would take the same free leader in the same step, the one nearest the leader (smallest g) wins.
- **Field swimmer that is being followed meets a faster swimmer** [decided]: it releases its own follower, which could be the agent, and starts following.

### Following motion
- **Lock behind** [decided]: the follower's position is leader position − g·(cos ψ_lead, sin ψ_lead). Its heading is set to ψ_lead, and its own noise and bias are off while it's locked [decided]. Entering the draft keeps the current gap g [decided], except that a gap closer than g_nom is set to g_nom [default]. The lateral offset is dropped when it locks on [default].
- **Gap update** [decided: catch-up at (1+β)v], applied per segment of length h:

  g′ = max(g_nom, g + a_lead − (1+β)·v·t_swim)

  Here a_lead is the leader's through-water distance over the segment: v_lead·h if it was swimming, 0 if it was pausing to sight. t_swim is the follower's swim time in the segment: h if swimming, 0 if pausing. During the agent's τκ pause the leader keeps going, so a sight opens a gap of about v_lead·τκ [decided].
  - This reproduces min(v_lead, (1+β)v): a follower with (1+β)v ≥ v_lead holds g_nom, and a slower one drifts back.
  - The draft is lost when g′ > g_max.
- **Release** [decided]: on release the follower keeps the leader's heading, so it inherits the leader's navigation error, and its own noise and bias resume.
- **Release triggers**: the follower chooses solo, g′ > g_max, the leader finishes, the leader starts following someone else (field rule above), or a field swimmer's draft duration expires.

### 4.1 Inside `advance(world, T)` [default]

**Start of the agent's step** (in `gen`, before calling `advance`): apply the agent's action. Choosing solo releases it; if it's solo, the turn is applied; a sight is scheduled to start after τ₀. A request to follow goes into the first boundary's draft pass alongside the field's requests, so nearest-wins applies to the agent too [default].

**Segments.** The interval [t, t + T] is split at every scheduled event:
- the agent's sight pause starting and ending
- each field swimmer's sight pause starting and ending
- field draft timers and cooldowns expiring

It is also split at contact-check ticks at most h_max apart, so contacts can't be missed between events.

**Within each segment, in order:**
1. Move solo swimmers (§3).
2. Move the agent if it's following; its leader is solo and has already moved.
3. Move field swimmers following solo field swimmers.
4. Move field swimmers following the agent.
5. Check finishes along each swimmer's path in the segment. If the agent finishes, stop at once and return the exact finish time.

**At each segment boundary:**
1. Releases: gap over g_max, leader finished, leader started following, duration expired.
2. New drafts, with the nearest-wins tie-break.
3. Releases again, so a leader that just started following can't leave an illegal chain.
4. Start sight pauses that are due. This comes last so that a field swimmer released in steps 1–3 sights in the same instant.

Pauses end when their timer runs out during the segment. A field swimmer re-aims at the buoy at that moment.

Swimmers react to each other at every boundary, not once per agent step.

## 5. Field swimmers

- **Sighting** [decided]: every N_k strokes, with each swimmer's schedule offset at random [decided]. Implemented as a countdown timer in seconds. When it expires, the swimmer pauses for τκ at that exact time, even partway through the agent's step.
- **Re-aim** [decided]: at the end of the pause, heading ← true bearing to the buoy from the swimmer's current position + N(0, σ_aim²). Field swimmers don't compensate for the current [decided: keep the field naive], so in a cross-current they swim a curved path well off the line.
- **Asymmetry** [default, deliberate]: a field swimmer re-aims as soon as its sight pause ends, while the agent's sight only informs its next turn.
- **Drafting rule** [decided]: draft any faster swimmer (v_lead > v_self) they come into contact with, for their own fixed duration. Afterwards there is a cooldown before they may draft again [decided]. A field swimmer doesn't start a draft during its own sight pause [default].
- **While following** [decided]: no sighting. On release, a sight is due immediately.
- **Per-swimmer variation** [decided]: each field swimmer has its own pace, heading bias, N_k and draft duration.
- **Finishing** [default]: a field swimmer that finishes is frozen at the buoy and drops out of the race. Its follower is released.

## 6. Agent

- **Pace** [decided]: a fixed parameter, while field paces are random around it. The default equals the field mean; the agent's pace relative to the field is an evaluation condition [decided], since it sets how many swimmers are worth drafting.
- **Actions** [decided]: sight m ∈ {0, 1}, continuous turn δ, follow/solo.
- **Follow** [decided]: follows the nearest eligible leader. The agent can't pick which one. Unlike field swimmers, it may follow slower swimmers.
- **No re-draft cooldown** for the agent [decided].
- **Action robustness** [default]: `gen` must accept every action in every state, because planners sample actions using particles that disagree about the draft status.
  - "Follow" with no eligible leader means swim solo.
  - "Follow" while already following keeps the current leader.
  - δ is ignored while following.
  - Sighting is always allowed.

## 7. Termination and reward

- **Finish** [decided]: the agent comes within r of the buoy, checked along the segment swum this step so it can't skip past.
- **Time cap** [decided]: the episode ends at t_cap with a penalty [default] of the remaining distance ÷ agent pace.
- **Reward** [default]: −(time elapsed in the step), which is Δt, or less on the step in which the agent finishes. On timeout, add −penalty. The episode ends when the agent finishes, whatever the field is doing.
- **Discount** [default]: steps differ in length (τ₀ vs τ₀ + τκ). A plain per-step γ < 1 would discount a sight step the same as a normal one. Use γ = 1, relying on the time cap to keep episodes finite; if a planner needs γ < 1, discount by γ^Δt.
- `gen` returns `sp` and `r`. The observation comes from `observation.jl`.

## 8. What lives where

- **Params** (fixed per problem): every constant and every distribution in §9.
- **State** (sampled per race in `initialstate`, carried through `gen`) [default]:
  - elapsed time and the current c
  - agent: position, heading, mode, leader index, gap, and its bias b
  - each field swimmer:
    - dynamic: position, heading, mode, leader index, gap, sight countdown, draft timer, cooldown timer, finished flag
    - per-race constants: pace, bias, sight period, draft duration

  Per-race constants live in the state, not the params, so the particle filter can be uncertain about them and the oracle can see them. Who follows whom can be derived from the leader indices.
- **Randomness**: only the `rng` passed to `gen` and `initialstate` is used, so common random numbers work.

## 9. Default values (proposed, please review)

| Symbol | Meaning | Proposed default | Units |
|---|---|---|---|
| L | buoy distance | 1500 | m |
| y₀ range | start spread | U(−20, 20) [decided] | m |
| r | finish radius | 3 | m |
| t_cap | time cap | 3 · L / v | s |
| v | agent pace | 1.3 | m/s |
| v_k | field paces | N(1.3, 0.1²), clipped to [1.0, 1.7] | m/s |
| τ₀ | cycle time (3 strokes) | 2.5 | s |
| τκ | sight time cost | 1.0 | s |
| δmax | max turn per cycle | 30° | rad |
| ψ₀ error | initial aim error | N(0, 5°²) | rad |
| σψ | heading noise | 1°/√s | rad/√s |
| b | heading bias per swimmer | N(0, (0.15°/s)²) | rad/s |
| σw | position noise | 0.2 | m/√s |
| ‖c‖ | current speed | U(0, 0.3) | m/s |
| ∠c | current direction | U(45°, 135°), mirrored to the other side with prob. ½ (mostly cross-course) [decided] | rad |
| σ_aim | field re-aim error | 5° | rad |
| β | draft speed boost | 0.20 [decided] | — |
| g_nom | nominal draft gap | 0.5 | m |
| g_max | max draft gap | 2.0 | m |
| w_lat | draft zone half-width | 0.5 | m |
| K | field size | 10 | — |
| N_k | field sight interval | U{6, …, 12} | strokes |
| field draft duration | per swimmer | U(60, 300) | s |
| field draft cooldown | after a draft | 60 | s |
| h_max | max segment length inside `advance` | 0.5 | s |

## 10. Path to multiple adaptive swimmers (not in v1)

`advance` already handles each swimmer's events at its own times, so moving to several decision-makers needs only a scheduler around it, not new dynamics:

- Every swimmer has a next decision time and a policy (rule-based, or adaptive).
- The scheduler advances the world to the earliest decision time, asks that swimmer's policy for an action, and repeats.
- One adaptive swimmer's POMDP becomes "advance until my next decision". The v1 `gen` is the special case where only the agent decides and field swimmers' rules run inside `advance`.

The practical limit is computation, not the clock. In a best-response loop, the other swimmers' policies run inside the planner's simulations millions of times. Those policies therefore have to be cheap, e.g. tuned rule parameters or a policy fitted to planner runs, not planners themselves.
