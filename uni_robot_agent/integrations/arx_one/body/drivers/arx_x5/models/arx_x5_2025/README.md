# ARX X5 2025 model resources

`x5_2025.urdf` is the portable copy of
`../../ros/src/arx_x5_controller/x5_2025.urdf`. Only the 14 mesh URI prefixes
change to `./meshes/`; joints, origins, axes, limits and inertial parameters remain
unchanged. Check it with:

```bash
python -m uni_robot_agent.integrations.arx_one.body.drivers.arx_x5.models.export --check
```

The kinematic chain is `base_link -> joint1 ... joint6 -> link6`, in metres and
radians. It contains no measured TCP, camera extrinsics or dual-arm installation
transform. Controller EEF coordinates are not automatically URDF/TCP coordinates.

## Meshes

The seven STL meshes are not distributed here. They are the X5 meshes of
[arx5-sdk](https://github.com/real-stanford/arx5-sdk) (MIT, see `licenses/`),
`models/meshes/X5/*.STL` at commit `c160db7834530a3d80fec524547c74e1d4fa8035`,
byte for byte. Place them in `meshes/` next to `x5_2025.urdf` and check them
against `SHA256SUMS`:

```bash
cd uni_robot_agent/integrations/arx_one/body/drivers/arx_x5/models/arx_x5_2025
mkdir -p meshes
for name in base_link link1 link2 link3 link4 link5 link6; do
  curl -fsSL -o "meshes/$name.STL" \
    "https://raw.githubusercontent.com/real-stanford/arx5-sdk/c160db7834530a3d80fec524547c74e1d4fa8035/models/meshes/X5/$name.STL"
done
sha256sum -c SHA256SUMS
```

Kinematics, preflight and the Body use only the URDF; the meshes are needed for
visualization.

The arx5-sdk X5 meshes are visual references. Modified fingers, brackets or mounts
need separate geometry verification. Upstream licenses remain in `licenses/`;
`SHA256SUMS` records the portable model and mesh bytes. Hardware profiles verify
the separate original ROS URDF hash, not the URI-rewritten portable file hash.
