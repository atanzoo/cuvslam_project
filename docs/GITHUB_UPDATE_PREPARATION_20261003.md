# GitHub update preparation — 2026-10-03

## Scope and status

The user requested an update of `https://github.com/atanzoo/cuvslam_project`
from this local folder and confirmed ownership of the `atanzoo` account.
The initial authentication blocker on October 3 was resolved through the
GitHub connector. On October 4, the connected identity was verified as
`atanzoo` with push access to this private repository. Remote `main` and local
HEAD both resolve to `9043a718bce6ac24ae21e5fd1abe7a0e0b52f961`.
HTTPS command-line Git authentication remains unavailable; publication uses
the authorized connector instead. The research snapshot was published and
verified on October 4; `main` remains unchanged and the draft is not merged.

Publication receipt:
- Branch: `codex/github-workstream-update-20261004`.
- Initial snapshot commit: `0793de37a49519485c029848044eb3eb3a549fa1`.
- Verified 546-file tree: `74200a28f4f45f773f6a31bccb16fff1a7b58268`.
- Draft review: `https://github.com/atanzoo/cuvslam_project/pull/1`.
- Remote branch SHA and README blob were read back successfully; remote
  `main` was read back at the unchanged base SHA above.
- A documentation-only follow-up records this receipt and corrects the
  handoff's GitHub metadata without changing technical milestone status.
- Local working files and ignored evidence remain intact. Command-line Git
  authentication is still unavailable: publication used the connector, and
  the local checkout stays on `main` with its snapshot staged, not a synced
  local commit or switched research branch.

This preparation is C0 documentation/ignore-policy work. It does not approve
or introduce runtime architecture changes. Existing workstream source and
research changes are preserved for subsequent review. Nothing has been
deployed or started on the Jetson during this task.

## Authorized snapshot publication — October 4

The user explicitly authorized upload and continuation. The target branch is
`codex/github-workstream-update-20261004`; `main` will remain unchanged.
This is publication of the existing local research snapshot, not approval
of the underlying incomplete runtime workstreams. The large migration spans
`real_robot/`, `simulation/`, `shared/`, and `research/`, plus their indexed
documentation and curated evidence. Old tracked locations are replaced by
the current folder layout; ignored raw data is retained locally.

This snapshot is intentionally larger than the normal implementation-commit
limit: splitting the existing relocation into partial uploads would leave
cross-workstream paths inconsistent. Review its complete Git tree and draft
pull-request diff before considering any merge. No new algorithm, runtime
deployment, or safety behavior is implemented by this publication operation.
Rollback is to close the draft request and leave the research branch unmerged;
the unchanged `main` and local ignored evidence remain available.

## Preparation changes

- `README.md`: describe quadruped navigation motivation before the cuVSLAM
  odometry work; separate real-camera results, experimental mapping,
  navigation proxy simulation, and incomplete dynamic-interaction gates.
- `.gitignore`: cover moved local environment files, raw traces, session
  captures, logs, binary costmap dumps, Gazebo pose dumps, downloaded R2
  recordings, and the generated robot-model archive.
- `docs/GITHUB_REPOSITORY_GUIDE.md`: correct current configuration paths and
  distinguish public source/curated summaries from private generated data.
- `docs/PROJECT_LAYOUT.md`: describe the actual remaining launch entry points
  instead of pointing to two removed root launchers.
- `shared/config/local.env.example`: correct the destination-path comment;
  placeholder values remain unchanged.
- `docs/README.md`: index this publication-preparation record.

No algorithm, safety setting, map, sensor calibration, or hardware behavior
was changed. Application PDFs and the portrait under `output/` remain ignored.
All local evidence remains available; exclusion from future commits is not
local deletion.

## Local inspection

After the ignore-policy correction, the candidate inventory contained 545
existing files, approximately 36.2 MB, before adding this record. This is an
inventory, not an approved staged manifest. Of 341 missing old tracked paths,
269 have byte-identical files at new workstream locations. Others include
modified relocated source, replaced documentation, removed root launchers,
and raw evidence intentionally excluded at its new location.

A bounded text scan found no private-key blocks, GitHub-token patterns,
AWS-access-key patterns, or credential-bearing HTTP URLs in that candidate
inventory. Password-assignment matches were reviewed as environment lookups,
status labels, or explicit placeholders. This scan is not a guarantee that
every possible sensitive value or license issue has been found. The final
staged manifest still needs review before publication.

Ignore checks confirmed protection for `shared/config/local.env`, a large
real-camera JSONL trace, the generated URDF ZIP, and the application portrait.
No `output/`, local environment file, raw JSONL/session capture, ZIP archive,
or PDF was present in the candidate inventory.

## Checks run

October 4 staged-snapshot recheck: 546 files; 188 Python AST checks and
15 shell syntax checks passed. The complete staged whitespace check also
identified pre-existing Markdown hard breaks and trailing whitespace in
relocated references/evidence. These are preserved rather than silently
rewriting the user's historical material; the earlier preparation-only
`git diff --check` result below is not a claim that this full snapshot is
whitespace-clean. No forbidden generated/private paths or bounded credential
patterns were found in the candidate inventory. Git-normalized staged blobs
are used for publication and server-side tree-hash comparison.

Environment: local macOS checkout, Homebrew Python 3.14.6 project environment,
NumPy 2.5.1 and MuJoCo 3.11.0. The connection-GUI unit test was additionally
run with `/usr/bin/python3`, because the project Python lacks `_tkinter`.

- Python AST parsing: all 188 candidate Python files passed.
- Bash syntax checking: all 15 candidate shell/launcher files passed.
- `git diff --check`: passed after the preparation edits.
- Local links in the root README and repository guide: existing targets.
- 37 offline test entry points were exercised: 36 passed after environment
  corrections; one legacy decision-layer entry point failed.

The entry points covered real odometry and A2M12 static contracts,
experimental-localization contracts, mocked connection recovery, all 19
cuVSLAM tool test files, all three `nav2_map3` test files, candidate maps,
robot geometry/profile, map export/path/costmap validation, map planning,
scenario/footprint checks, MuJoCo model loading/stepping, and the legacy
decision layer. The interaction-input contract ran 33 focused unit tests.

The D1 Max geometry test initially skipped because it was launched from its
tools directory and resolves the ignored model relative to the working
directory. Rerunning from the repository root exercised its assertion and
passed. The mocked connection recovery test passed three cases with the
system Python; no live connection recovery was performed.

The remaining failure is
`simulation/path_planning/tools/test_d1_edu_decision_layer.py`: the crossing
scenario reported minimum clearance `-0.056 m` and failed the final `CRUISE`
assertion. It was not fixed or hidden. This is not a fully passing navigation
baseline and should not be silently promoted to `main` as one.

## Not run / next step

No ROS launch, ARM64 custom-critic build/load, bag replay, Jetson deployment,
live sensor test, real-robot motion, or hardware navigation acceptance was
performed. Not all available ML/training tests were run.

Review the final staged changes and publish the existing
research snapshot on a `codex/` branch for review while keeping the failing
legacy gate and incomplete runtime gates explicit. Do not force-push or
overwrite remote changes. Rollback of this preparation is limited to its
document/ignore edits; preserve all pre-existing user changes and local data.
