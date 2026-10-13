# Runtime path rules

No runtime code may assume a developer-specific absolute checkout path. Scripts derive the repository root
from their own location. Launch files use package shares and explicit launch
arguments. Route assets are resolved relative to `route.yaml`.

For physical motion, set `G1_LOCO_CLIENT_BIN`, `UNITREE_SDK_LIB_DIR`, and
`UNITREE_SDK2_ROOT` in the environment. The default profile keeps
`enable_motion:=false`.
