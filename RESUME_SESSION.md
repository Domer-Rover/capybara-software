# Resuming the planning session with Claude Code

Run from this directory (memory is keyed to it):

```bash
cd ~/Documents/domerrover/capybara-software
claude --resume 07efeab0-29a2-497d-ba99-0fa72a266c3d
```

If that id is not found, run `claude --resume` with no argument and pick the
session from 2026-09-07/08 that starts "This is the main repo for the main
robot". Web link to the same session:
https://claude.ai/code/session_01XooinpebaTuuvn3FGVN3Cu

## Where we left off

- Season plan: `../ROADMAP.md`
- Done: `scripts/add_dev_user.sh` + Developer Accounts section in `DOCUMENTATION.md`; housekeeping edits (uncommitted, branch `nav2`).
- Next: test `add_dev_user.sh` on the Jetson, then Sprint 1
  (`wheel_separation` fix, HW interface hardening, udev rules, VIO drift test).
