# The test console

A page for testing policies on the D1 dog: the list of policies, an E-stop, and what the running program reports.

```bash
demos/robot_console/run_on_pc.sh      # sends this tree to the dog, starts the console there, opens http://localhost:8097
```

Ctrl-C in that terminal stops the console. Records come back with `d1fetch`, then `./dashboard.py record` as usual.

**A PC without the `d1*` helpers** (they are in `~/.bashrc`, not the repo): name the dog with `--dog`. There are two dogs and the host key tells them apart, so it has to be in `known_hosts` first: a new one is refused here, never accepted. Compare it with `ssh-keygen -F p2dingo-go2 | awk 'NF>2{print "x",$2,$3}' | ssh-keygen -lf -` on a PC that has it, and this PC's own key has to be on the dog.

```bash
demos/robot_console/run_on_pc.sh --dog unitree@192.168.1.213 --preview   # what would change on the dog; sends nothing
demos/robot_console/run_on_pc.sh --dog unitree@192.168.1.213             # sends this tree, starts the console, opens the page
```

The copy has `--delete`, so read the preview: the dog's tree becomes this tree, whatever another PC sent it. `logs/`, `results/`, `generated/`, `.git/` and the `.pt` checkpoints stay. One console at a time, and no copy during a run.

## The E-stop

The button, **Esc**, or **B on the remote**. It puts the motors in damping, stops LowCmd and selects sport mode at once.

| what fails | what stops the run |
| --- | --- |
| nothing | B (read by the program from `LowState`), the button, Esc |
| the browser, the PC or the WiFi | B; the server after 3.5 s without a heartbeat: a whole-body run with its arm out or coming out ends as End gently does, one already folding is left to finish, anything else is the E-stop (F-262) |
| the link lost, and the program not reporting | the E-stop |
| the tunnel or the server | B; the program's stdin closing |
| the program does not answer | SIGTERM to its container after 0.6 s: damping |
| the program ends with sport mode released, or dies | `run_sport_on_dog.sh --ensure` |

The button with no run going checks sport mode and selects it only if it is released.

**An E-stop from a live stand** has happened twice, from the link deadman (F-262, 2026-11-07): sport mode confirmed 3.1 s later, and it stood the dog up by itself with the arm out. During that time the dog is in damping, and the record ends at the stop, so how it sinks is not in it. B has stopped dry runs only.
If sport mode is not confirmed the page says so, and the motors stay in damping.

## Live is locked until

In one console session, for that policy:
1. B has been seen, in any dry run (the chip says so), from the program's status or its log: B ends a run before a status can carry it.
2. The policy has run dry for at least 5 s with the dog standing, ending cleanly or on B.

Then Live starts the program, which checks the dog, and `live` is typed at its confirmation (or A on the remote). On **Reach sticky 66600** (`"quick_start": true`) nothing is typed: one OK on the page stands for `arm` and for `live`, which the console sends once the program has checked the dog and asks for it. The Live lock above still applies. Other policies keep their words.

## Far goals and walking

On **Reach sticky 66600**, each started with the page's OK. The order and the checks are in [the robot README](../combiner_key/robot/README.md#far-goals-and-walking-on-the-whole-body---reach_via---reach_walk-prepared-2026-11-07). **Do not touch the remote's sticks in a dry run:** sport mode has them there and walks the dog. **Stick check** is a live stand that reads them and walks nothing, and comes before any walking run. A walking run stops when the sticks are released; let go of them when pressing B too, since B hands them back to sport mode.

## Wiping a bench

**Wipe: high** to **Wipe: hard** on **Reach sticky 66600**. The jaws wait open with the arm out: hand it the cloth, then A or **Continue** shuts them. Then the sticks walk it to the bench, and a second A takes them away and starts the wipe. Setup and the order are in [the robot README](../combiner_key/robot/README.md#wiping-a-bench---reach_cloth-prepared-2026-11-07).

## Wiping with the force input

**Force check** and **Force wipe: about 6, 11, 14, 19 N** on **Reach sticky 66600**. The policy is told a force, and nothing regulates the real one. [The robot README](../combiner_key/robot/README.md#wiping-with-the-force-input---reach_force_n-prepared-2026-11-07) has the steps and what they measured.

## Runs that move the arm

**Arm zero test** (`kind: arm`, `run_holster_on_dog.sh --zero_test --console`) moves the arm and never the legs, so its stop
is not the table above. The button, Esc, B, or the link lost: `a`, the arm held where it is, sport mode left alone, the
program kept up. **Finish** or A then folds it by home, `tuck`, `fold`. It has no `q`: that quits with the arm out. Its
stdin closing is a stop, and A on the remote can still fold it. If the program itself dies the arm stays held and out, and
the console cannot fold it. Live is locked until B has been seen and the last dry run ended finished or stopped by the
operator with the real arm folded and its report fresh. The dog need not stand: sit it.

**Reach locked 66200** (`--reach_arm_send`) drives the legs and the arm, with the stop of the table above: damping with the
arm held out, then sport mode. `press_live.py` asks for `arm` on its stdin before it starts, so the console takes the word
on the page and passes it down.

A mode in a policy's `confirm` needs its word typed on the page before it starts, on top of `live` where the program asks.

## Adding a policy

`policies.json`: its export, a status line, and the exact arguments of each mode. A mode without `--live` is a dry run.
Rejected policies keep a `dry` mode only. Export reach and leg policies with `demos/combiner_key/robot/export_policy.py`.

## How it works

`server.py` runs on the dog's host, standard library only, on 127.0.0.1, so it is reached through the key-checked SSH
tunnel and is not open on the WiFi. POSTs need a custom header, so another web page cannot start a run. It runs
`run_press_on_dog.sh` as a child: commands go down its stdin, and `press_live.py --status_port` sends status back as
datagrams, which cannot hold up the 50 Hz loop if the console is gone.
