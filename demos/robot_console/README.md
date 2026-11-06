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
B has never stopped a run. Sport mode taking a sinking dog is untried: every handback so far was from lying or settled
damping, ~3.2 s to confirm, and selecting it may stand the dog up by itself. During that time the dog is in damping.
If sport mode is not confirmed the page says so, and the motors stay in damping.

## Live is locked until

In one console session, for that policy:
1. B has been seen, in any dry run (the Keys tile lights).
2. The policy has run dry for at least 5 s with the dog standing, ending cleanly or on B.

Then Live starts the program, which checks the dog, and `live` is typed at its confirmation (or A on the remote).

## Adding a policy

`policies.json`: its export, a status line, and the exact arguments of each mode. A mode without `--live` is a dry run.
Rejected policies keep a `dry` mode only. Export reach and leg policies with `demos/combiner_key/robot/export_policy.py`.

## How it works

`server.py` runs on the dog's host, standard library only, on 127.0.0.1, so it is reached through the key-checked SSH
tunnel and is not open on the WiFi. POSTs need a custom header, so another web page cannot start a run. It runs
`run_press_on_dog.sh` as a child: commands go down its stdin, and `press_live.py --status_port` sends status back as
datagrams, which cannot hold up the 50 Hz loop if the console is gone.
