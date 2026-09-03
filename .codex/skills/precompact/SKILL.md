---
name: precompact
description: Use right before running /compact (or whenever context is about to be lost/summarized) to save a snapshot of current architecture understanding, data/workflow flow, and exactly what's in progress right now. Triggers on "lưu context trước khi compact", "precompact", "sắp compact rồi", "trước khi nén hội thoại". Manual step — Codex does not auto-run this on /compact.
---

# Precompact

Codex's built-in `/compact` summarizes the conversation but loses the fine-grained state that's
easy to re-derive while the context is live and expensive to re-derive after compaction. This skill
writes that state to `CONTEXT.md` (repo root) right before it's lost, so the next session (post-
compact, or a fresh session entirely) can resume without re-reading the whole conversation or
re-exploring code that was already understood.

This is a **snapshot of the current moment**, not a history log — unlike a permanent incident-history
doc (if this repo keeps one), `CONTEXT.md` is expected to be **overwritten each time this skill runs**,
not appended to. Stale content here is actively misleading, not just unhelpful.

## What to write into `CONTEXT.md`

Overwrite the whole file with these sections, in this order:

0. **Header** — one line at the very top: snapshot timestamp (date + time, from `date` — don't guess)
   and, if a prior `CONTEXT.md` existed, a one-line carry-over pointer ("continues from the snapshot
   taken at <old timestamp>, which covered <1-clause summary>"). This is how a future session judges
   whether the snapshot is still fresh enough to trust, and links a chain of snapshots across multiple
   compactions instead of each one reading as if it's the first.
1. **Architecture understanding right now** — only what's relevant to the current work and NOT
   already stable/documented elsewhere (a project AGENTS.md, package READMEs). If nothing new was
   learned about architecture this session, say so in one line and point to where the stable docs
   live instead of repeating them.
2. **Data/workflow flow currently relevant** — if this session touched a specific path through the
   system (e.g. "the sim-side state-estimation path from /imu/data through MegadogWbcRuntime" or "the
   CAN-FD frame build for kp/kd"), describe that specific path concretely (files, functions, in what
   order) — not the whole system diagram.
3. **What's in progress right now, in detail** — the single most important section. Be concrete:
   - What was the user's most recent request, verbatim intent (not just a category).
   - What has been done so far toward it (files touched, decisions already made/confirmed by the
     user - don't re-litigate these after resuming).
   - What is NOT done yet, and the exact next step (not "continue implementing X" - say which file,
     which function, which line, what change).
4. **Blockers & open decisions** — anything genuinely stalled or waiting on the user, listed
   separately from ordinary "not done yet" work so it isn't buried. For each: what's blocking it, and
   what answer/action would unblock it. If nothing is blocked, say so in one line.
5. **Files modified this session** — a flat list of every file created/edited/deleted, one line each
   (path + one-clause what changed). This is the index a resuming session uses to decide what to
   re-read before touching anything nearby, instead of re-deriving it from `git diff`/`git status`
   (which won't show changes already committed, or won't be checked at all if the resuming session
   doesn't think to run it).
6. **Environment state** — current git branch, whether the working tree had uncommitted changes at
   snapshot time (from `git status`), and — **only relevant for this repo since it drives real
   motors** — whether any real-hardware process was known to be running/flashing at snapshot time.
   Get this from actual commands, not memory of earlier commands run mid-session (state may have
   changed since). Existing purely to prevent a resumed session from assuming the wrong branch or
   acting as if hardware is idle when it might not be.
7. **How to use this file** — a fixed, short paragraph (same wording every time) telling a resuming
   session: read this file first, verify anything load-bearing before acting on it (per "Before
   recommending from memory" - file paths/functions named here may have moved on), treat section 3-4
   as the actual task list, and overwrite this file again before the next compaction rather than
   appending.
8. **Anything explicitly said but not yet acted on** — e.g. a preference stated mid-conversation that
   hasn't been reflected in code/docs yet.

## Process

1. Read back through the current conversation (don't guess from memory of memory - re-derive from
   what's actually been said and done this session).
2. Check whether `CONTEXT.md` already exists and read it first - if it does, extract its header
   timestamp/summary for the new carry-over pointer, then this is a full replacement (see "snapshot,
   not log" above), not a merge.
3. Run `date`, and `git branch --show-current` + `git status --short` (repo root) to get real values
   for sections 0 and 6 - do not fill these from memory.
4. Write the new `CONTEXT.md`.
5. Tell the user briefly what was captured (1-2 sentences) so they can correct it before running
   `/compact` if something's off - this file only gets one shot at being right before context is gone.

## Do not

- Do not duplicate stable content already documented elsewhere in the repo - link to it instead.
- Do not write speculative next steps the user hasn't actually indicated - "what's in progress" means
  what was actually being worked on, not a wishlist.
- Do not skip this because "nothing much happened" - even a short confirmed decision is worth one line
  so it isn't silently dropped.
