# MAVLink Routing Compliance Test

- fc_system_id: `1`
- links: `['link_camera', 'link_companion', 'link_gimbal', 'link_radio']`
- gcs_link: `link_radio`
- components: `[('mav2_companion_computer', 1, 191, 'link_companion'), ('mav1_elrs_receiver', 1, 68, 'link_radio'), ('mav3_camera', 1, 100, 'link_camera'), ('mav4_gimbal', 1, 154, 'link_gimbal')]`

## Rule References

- broadcast_forward: `routing.md:25-27,48-49`
- known_target_forward: `routing.md:49-50; MAVLink_routing.cpp:66-80`
- unknown_target_blocked: `routing.md:26-27,53`
- no_ingress_loop: `MAVLink_routing.cpp:197-209`
- no_repack: `routing.md:29-31`

## Results

| Case | Source(link) | Target(sys,comp) | Expected Links | Observed Links | Routing | No Ingress Loop | Payload Intact | Pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `gcs_broadcast` | `gcs (link_radio)` | `(0,0)` | `['link_camera', 'link_companion', 'link_gimbal']` | `['link_camera', 'link_companion', 'link_gimbal']` | `True` | `True` | `True` | `True` |
| `gcs_target_local_system_component_broadcast` | `gcs (link_radio)` | `(1,0)` | `['link_camera', 'link_companion', 'link_gimbal']` | `['link_camera', 'link_companion', 'link_gimbal']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav2_companion_computer` | `gcs (link_radio)` | `(1,191)` | `['link_companion']` | `['link_companion']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav1_elrs_receiver` | `gcs (link_radio)` | `(1,68)` | `[]` | `[]` | `True` | `True` | `True` | `True` |
| `gcs_target_mav3_camera` | `gcs (link_radio)` | `(1,100)` | `['link_camera']` | `['link_camera']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav4_gimbal` | `gcs (link_radio)` | `(1,154)` | `['link_gimbal']` | `['link_gimbal']` | `True` | `True` | `True` | `True` |
| `gcs_target_unknown_component` | `gcs (link_radio)` | `(1,199)` | `[]` | `[]` | `True` | `True` | `True` | `True` |
| `gcs_target_unknown_system` | `gcs (link_radio)` | `(42,1)` | `[]` | `[]` | `True` | `True` | `True` | `True` |
| `mav2_companion_computer_broadcast` | `mav2_companion_computer (link_companion)` | `(0,0)` | `['link_camera', 'link_gimbal', 'link_radio']` | `['link_camera', 'link_gimbal', 'link_radio']` | `True` | `True` | `True` | `True` |
| `mav2_companion_computer_to_mav1_elrs_receiver` | `mav2_companion_computer (link_companion)` | `(1,68)` | `['link_radio']` | `['link_radio']` | `True` | `True` | `True` | `True` |
| `mav2_companion_computer_to_gcs` | `mav2_companion_computer (link_companion)` | `(1,68)` | `['link_radio']` | `['link_radio']` | `True` | `True` | `True` | `True` |

summary pass_count=11 fail_count=0 total=11

