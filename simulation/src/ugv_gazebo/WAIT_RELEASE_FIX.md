# Multi-UGV WAIT release fix (V3)

The V2 run showed that agent0, agent2 and agent5 completed, while the three agents containing LCBR WAIT actions remained pinned to their first dense WAIT sample:

- agent1: WAIT block dense indices 20..79, release index 80, LCBR interval [1,4)
- agent4: WAIT block dense indices 140..199, release index 200, LCBR interval [7,10)
- agent3: WAIT block dense indices 480..519, release index 520, LCBR interval [24,26)

Cause: a WAIT primitive is densified into many samples at the identical pose. After the WAIT interval expired, nearest-point tracking could remain on the first identical WAIT sample forever. The fallback `direction == wait` then kept publishing zero velocity indefinitely.

V3 adds `release_expired_wait(planner_time)`. When the current dense reference index is inside an expired WAIT block, it advances only the *reference index* to the first post-WAIT sample. The Gazebo vehicle pose is not modified. Normal Pure Pursuit then resumes, including normal direction-switch handling.

Expected release messages:

- `[agent1] WAIT RELEASE ... idx=20->80`
- `[agent4] WAIT RELEASE ... idx=140->200`
- `[agent3] WAIT RELEASE ... idx=480->520`

All V2 tracking, temporal-frontier, Ackermann and speed settings are otherwise retained. Gazebo target RTF remains 20 with max_step_size=0.01 and the controller remains at 200 Hz.
