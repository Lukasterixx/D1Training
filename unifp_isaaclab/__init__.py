"""Run UniFP's trained Go2+D1 policy on this repository's Isaac Lab model.

`unifp_go2d1/` trains the policy on the legacy stack (Isaac Gym Preview 4, Python 3.8). This
package runs the resulting checkpoint on the Isaac Lab welded Go2+D1 instead -- a sim-to-sim
transfer test, and the step before any question about hardware can be asked.

Nothing here imports Isaac Gym, and `interface.py` and `policy.py` import no Isaac Lab either,
so the contract and the network can be checked with no simulator running.
"""
