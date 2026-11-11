# Sources and boundaries

The Body 2.0 core and environment lifecycle are adapted from the ARX ONE
integration in Open-RUA/uni-robot-agent at
`d5e8085cebc05e59aaf9e237e24c7a59b1a59708`, under the repository MIT license.
The original ARX contribution is by Jiarui Yuan (`1rubbishyuan`), beginning with
[PR #28](https://github.com/Open-RUA/uni-robot-agent/pull/28).

Piper-specific adaptations retain group-only motion admission, physical resource
locks and a lightweight Python host. `body/` is scoped to this integration;
it does not change the ARX runtime or add shared services or model-facing tools.
The environment uses mainline whole-path and typed receipt contracts. Native
request reconciliation queries the original operation without repeating it.

Pose tracking and arrival derive from ARX ONE's MIT-licensed safety implementation
(initially PR #35, `731b110`), retaining pose tracking tolerances, lag allowance
and consecutive feedback confirmation. Native CAN feedback, limits, geometry
and joint-position holding remain Piper-specific. No ARX controller binaries,
ROS runtime or camera depth-completion service is included.

The versioned passive CAN decoder was compared against the official
[AgileX pyAgxArm SDK](https://github.com/agilexrobotics/pyAgxArm), revision
`e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b`. The SDK remains an external dependency;
no native control bridge or vendor binary is bundled. Each deployment pins its
installed SDK version, driver source hash, firmware profile and reported version.

No vendor SDK, binary, URDF, mesh, measured calibration or evaluation output is
bundled. Evaluation uses the official UNI CLI; there is no private task runner.
