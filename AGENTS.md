# AGENTS.md

Flight control law design in MATLAB/Simulink for the Su-35 mod's flight model (the mod lives in a separate private
repo, `su35-cws`; its `src/native/cpp/xfm/su35_xfm.cpp` is where the laws are hand-ported).

## Rules

- **This repo may go public.** Never commit parameters taken from the mod's flight model (they go in
  `plant/private/`, git-ignored), manuals, course material, or anything from Eagle Dynamics. Cite sources in
  `docs/sources.md` instead.
- **Commit or push only when the human asks.** No agent trailers in commit messages.
- **Plain code.** Readable MATLAB: named intermediate values, one idea per line, comments that say why. Units in
  names or comments (SI inside the models; km/h and degrees only at the edges).
- **MATLAB:** `~/matlab/bin/matlab -batch "setup; <command>"`. Save Simulink models as `.slx`, built from scripts
  where practical so the diff stays reviewable.
