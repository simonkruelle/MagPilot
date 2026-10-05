# Franka FR3 model

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/franka_fr3

Pinned revision: `4d038b3feae26ec82b46a4d586379114012a8ac7`.

`fr3.xml`, `LICENSE`, and all model-referenced meshes inside `meshes.zip` are unmodified upstream files. Meshes are compressed to keep the repository size manageable; the backend supplies them directly to MuJoCo. See `UPSTREAM_README.md` for model derivation and `LICENSE` for Apache-2.0 terms.

The MagPilot scene adds a non-colliding task target, workspace wireframe, and fixed pick-and-place markers without modifying the source model.
