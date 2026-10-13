# Published map

The published map is `maps/site_04_nav_zup_ccw90/`. `prior.pcd` is the
localization map and `site_04_compact_trajectory.vgh` is the FAR graph input.
Use `lidar_inverted:=true` and `route_planner_config:=site04_zup`.

Run `sha256sum -c routes/nv_zup_ccw90/map.sha256` from the repository root.
The hashes are also recorded in `conversion.json`. The recorded
`field_alignment_verified: false` value is intentional: the geometry and
offline replay are consistent, while field absolute alignment remains a
separate acceptance check.
