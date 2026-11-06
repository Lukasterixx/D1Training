# The test console

A page for testing policies on the D1 dog: the list of policies, an E-stop, and what the running program reports.

```bash
demos/robot_console/run_on_pc.sh      # sends this tree to the dog, starts the console there, opens http://localhost:8097
```

Ctrl-C in that terminal stops the console. Records come back with `d1fetch`, then `./dashboard.py record` as usual.

## The E-stop

The button, **Esc**, or **B on the remote**. It puts the motors in damping, stops LowCmd and selects sport mode at once.

| what fails | what stops the run |
| --- | --- |
| nothing | B (read by the program from `LowState`), the button, Esc |
| the browser, the PC or the WiFi | B; the server after 2 s without a heartbeat |
| the tunnel or the server | B; the program's stdin closing |
| the program does not answer | SIGTERM to its container after 0.6 s: damping |
| the program ends with sport mode released, or dies | `run_sport_on_dog.sh --ensure` |

The button with no run going checks sport mode and selects it only if it is released.

**Not validated on the dog:** an E-stop from a live stand. Every stop above has run dry, sending nothing (Week 8 log), and
B has stopped dry runs only (2026-11-06). Sport mode taking a sinking dog is untried: every handback so far was from lying or settled
damping, ~3.2 s to confirm, and selecting it may stand the dog up by itself. During that time the dog is in damping.
If sport mode is not confirmed the page says so, and the motors stay in damping.

## Live is locked until

In one console session, for that policy:
1. B has been seen, in any dry run (the chip says so), from the program's status or its log: B ends a run before a status can carry it.
2. The policy has run dry for at least 5 s with the dog standing, ending cleanly or on B.

Then Live starts the program, which checks the dog, and `live` is typed at its confirmation (or A on the remote).

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
