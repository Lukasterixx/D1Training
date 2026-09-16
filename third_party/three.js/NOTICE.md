# three.js

Source: <https://github.com/mrdoob/three.js>, release **r128** (npm `three@0.128.0`), fetched from
`https://cdn.jsdelivr.net/npm/three@0.128.0/`. Licensed under the MIT License (`LICENCE` in this folder).

Vendored rather than loaded from a CDN because the browser console runs on the Go2's Jetson payload,
which is reached over Tailscale and has no dependable route to the public internet. The files are
unmodified.

| This repository | Vendored from |
| --- | --- |
| `d1_ui/static/vendor/three.min.js` | `build/three.min.js` |
| `d1_ui/static/vendor/OrbitControls.js` | `examples/js/controls/OrbitControls.js` |
| `d1_ui/static/vendor/ColladaLoader.js` | `examples/js/loaders/ColladaLoader.js` |

r128 is pinned deliberately: it is the last series in which `OrbitControls` and `ColladaLoader` ship as
plain scripts defining `THREE.*` globals. Later releases publish them as ES modules only, which would
require a bundler or an import map on a machine with no network access.
