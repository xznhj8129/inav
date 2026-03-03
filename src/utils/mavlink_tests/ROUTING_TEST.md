# MAVLink Routing Compliance Test

- fc_system_id: `1`
- links: `['link_camera', 'link_components_a', 'link_gcs', 'link_gimbal']`
- gcs_link: `link_gcs`
- components: `[('mav1_elrs_receiver', 1, 68, 'link_components_a'), ('mav2_companion_computer', 1, 191, 'link_components_a'), ('mav3_camera', 1, 100, 'link_camera'), ('mav4_gimbal', 1, 154, 'link_gimbal')]`

## Rule References

- broadcast_forward: `routing.md:25-27,48-49`
- known_target_forward: `routing.md:49-50; MAVLink_routing.cpp:66-80`
- unknown_target_blocked: `routing.md:26-27,53`
- no_ingress_loop: `MAVLink_routing.cpp:197-209`
- no_repack: `routing.md:29-31`

## Results

| Case | Source(link) | Target(sys,comp) | Expected Links | Observed Links | Routing | No Ingress Loop | Payload Intact | Pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `gcs_broadcast` | `gcs (link_gcs)` | `(0,0)` | `['link_camera', 'link_components_a', 'link_gimbal']` | `['link_camera', 'link_components_a', 'link_gimbal']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav1_elrs_receiver` | `gcs (link_gcs)` | `(1,68)` | `['link_components_a']` | `['link_components_a']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav2_companion_computer` | `gcs (link_gcs)` | `(1,191)` | `['link_components_a']` | `['link_components_a']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav3_camera` | `gcs (link_gcs)` | `(1,100)` | `['link_camera']` | `['link_camera']` | `True` | `True` | `True` | `True` |
| `gcs_target_mav4_gimbal` | `gcs (link_gcs)` | `(1,154)` | `['link_gimbal']` | `['link_gimbal']` | `True` | `True` | `True` | `True` |
| `gcs_target_unknown_component` | `gcs (link_gcs)` | `(1,199)` | `[]` | `[]` | `True` | `True` | `True` | `True` |
| `gcs_target_unknown_system` | `gcs (link_gcs)` | `(42,1)` | `[]` | `[]` | `True` | `True` | `True` | `True` |
| `mav1_elrs_receiver_broadcast` | `mav1_elrs_receiver (link_components_a)` | `(0,0)` | `['link_camera', 'link_gcs', 'link_gimbal']` | `['link_camera', 'link_gcs', 'link_gimbal']` | `True` | `True` | `True` | `True` |
| `mav1_elrs_receiver_to_mav3_camera` | `mav1_elrs_receiver (link_components_a)` | `(1,100)` | `['link_camera']` | `['link_camera']` | `True` | `True` | `True` | `True` |
| `mav1_elrs_receiver_to_gcs` | `mav1_elrs_receiver (link_components_a)` | `(255,190)` | `['link_gcs']` | `['link_gcs']` | `True` | `True` | `True` | `True` |

summary pass_count=10 fail_count=0 total=10

