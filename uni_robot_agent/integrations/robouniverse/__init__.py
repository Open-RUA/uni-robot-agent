"""Direct control of externally prepared RoboUniverse-Verified episodes.

This integration is intentionally minimal and uses Uni's standard ``DIRECT_TOOLS``
profile without modifying shared gripper or scene tool contracts. Any benchmark-
specific motion extensions (such as custom multi-waypoint tools or Harbor
wheelhouse packaging) belong in the external RoboUniverse runner harness rather
than inside ``uni-robot-agent``.
"""

# The Harbor container ships a raw ``roboenv_client`` alongside this bridge.
# Stepping through it would advance the simulator behind the adapter's back,
# so we forbid raw-client access and also preserve the physics-settling note
# that originally lived inside the stripped raw-client instruction section.
RULES = (
    " Never import or run roboenv or roboenv_client, and never read /opt/roboenv: "
    "the registered robot tools are the only way to act in this environment. "
    "Simulation advances ONLY when you execute robot actions; calling finish adds ZERO settling steps. "
    "After releasing or completing manipulation and withdrawing the arm clear of the workspace, "
    "run hold(steps=15..25) (at least ~0.75-1.0s at control_frequency_hz) and verify from the "
    "observation that objects are stationary and the hand is withdrawn before calling finish."
)


def get_integration_spec():
    from dataclasses import replace

    from uni_robot_agent.integrations.gripper import DIRECT_TOOLS
    from uni_robot_agent.integrations.simulation import build_simulation_spec

    spec = build_simulation_spec(
        "robouniverse",
        tool_profiles={"direct": DIRECT_TOOLS},
        extra_rules=RULES,
    )
    return replace(spec, debug_modules=(), allow_automatic_retry=False)
