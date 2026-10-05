# Franka Hand model

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/franka_emika_panda

Pinned revision: `4d038b3feae26ec82b46a4d586379114012a8ac7`. `panda.xml`, `LICENSE`, and the eight hand/finger mesh files in `meshes.zip` are unmodified upstream files. The archive uses an `assets/hand/` namespace to avoid collisions with the FR3 arm meshes.

MagPilot extracts the hand subtree, material names and contact-pad defaults at load time, attaches it to the FR3 flange, and fixes both fingers open with a 40 mm inner-pad gap. Named fingertip sites sit at the centre of the opposing inner pad faces. Their measured midpoint defines `gripper_center`; it is 102.9 mm along the hand axis from the flange. The fingers are fixed for this reaching-only task; grasping is not part of the protocol.

See `UPSTREAM_README.md` for derivation and `LICENSE` for Apache-2.0 terms.
