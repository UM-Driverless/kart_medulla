#!/usr/bin/env python3
"""Move done content out of kart-medulla's tasks.md into tasks/done-archive.md.

Adapted from partle's tasks/archive_done.py. Two differences from that repo:
  * headings here are `## TODO` / `## In Progress` / `## Done` (level 2, not `# Done`)
  * the `## Done` section holds OPEN `- [ ]` items as well as closed ones, so it
    cannot be moved wholesale -- open items go back onto the board.
"""
import re, sys

TASKS = "/Users/rubenayla/repos/hardware/kart-medulla/tasks.md"
lines = open(TASKS).read().split("\n")


def block_at(src, i):
    """The bullet at src[i] plus its indented / interleaved-blank continuation."""
    block = [src[i]]
    j = i + 1
    while j < len(src):
        nxt = src[j]
        if nxt == "":
            if j + 1 < len(src) and re.match(r"^\s+\S", src[j + 1]):
                block.append(nxt)
                j += 1
                continue
            break
        if re.match(r"^\s+\S", nxt):
            block.append(nxt)
            j += 1
            continue
        break
    return block, j


# --- split the file at `## Done` -------------------------------------------
done_start = next(i for i, l in enumerate(lines) if l.strip() == "## Done")
before, done_section = lines[:done_start], lines[done_start + 1:]

# --- from the Done section, separate closed bullets from open ones ----------
archived_from_done, reopened = [], []
i = 0
while i < len(done_section):
    l = done_section[i]
    if re.match(r"^- \[x\]", l) or re.match(r"^- \[\d{4}-\d{2}-\d{2}\]", l):
        blk, i = block_at(done_section, i)
        archived_from_done += blk
    elif re.match(r"^- \[ \]", l):
        blk, i = block_at(done_section, i)
        reopened += blk
    else:
        archived_from_done.append(l)
        i += 1

# --- which sections still have open work? ----------------------------------
# A `- [x]` under a heading that still holds `- [ ]` items is a finished STEP of a
# live task, not a finished task. Archiving it would strip the open step of the
# context that says what was already settled -- e.g. "Put the QR/label on the
# board" is meaningless once "Pick the scheme" has been filed away. So only
# sections with nothing open left are archived.
section_open = {}
heading = ""
for l in before:
    if re.match(r"^#{2,3} ", l):
        heading = l
        section_open.setdefault(heading, False)
    if re.match(r"^- \[ \]", l):
        section_open[heading] = True

has_closed = {}
heading = ""
for l in before:
    if re.match(r"^#{2,3} ", l):
        heading = l
        has_closed.setdefault(heading, False)
    if re.match(r"^- \[x\]", l):
        has_closed[heading] = True

kept, archived_from_board = [], []
heading = ""
i = 0
while i < len(before):
    l = before[i]

    # A fully-closed `###` section travels whole -- heading, prose and all --
    # so the archive keeps the reasoning and the board is not left with an
    # orphan heading over nothing.
    if re.match(r"^### ", l) and not section_open.get(l, False) and has_closed.get(l, False):
        j = i + 1
        while j < len(before) and not re.match(r"^#{2,3} ", before[j]):
            j += 1
        archived_from_board.append((None, before[i:j]))
        i = j
        continue

    if re.match(r"^#{2,3} ", l):
        heading = l

    # A bullet directly under a `##` heading (`## TODO`) is a standalone task
    # and travels on its own. Inside a `###` cluster, a done step stays put
    # until the whole cluster closes -- otherwise the remaining open step loses
    # the record of what was already settled.
    standalone = heading.startswith("## ") and not heading.startswith("### ")
    if re.match(r"^- \[x\]", l) and standalone:
        blk, i = block_at(before, i)
        archived_from_board.append((heading, blk))
        continue
    kept.append(l)
    i += 1

print(f"[x] blocks pulled off the board : {len(archived_from_board)}")
print(f"lines archived from ## Done     : {len(archived_from_done)}")
n_reopened = sum(1 for l in reopened if re.match(r"^- \[ \]", l))
print(f"open items rescued from ## Done : {n_reopened}")

if "--apply" not in sys.argv:
    print("\n(dry run -- pass --apply to write)")
    for h, b in archived_from_board:
        print(f"  {(h or b[0])[:90]}")
    sys.exit(0)

# --- write the archive -----------------------------------------------------
out = [
    "<!-- reference — read only when you need the history of a shipped item -->",
    "# Done archive — completed work items",
    "",
    "Closed items moved out of the root `tasks.md` on 2026-08-10, following the same convention as",
    "the partle repo. Nothing here is actionable: the root board carries only live work, while the",
    "reasoning behind finished things stays findable.",
    "",
    "The board is `tasks.md` at the repo root — the only task board in this repo. Narrative context",
    "for most entries is in `history.md` (append-only, newest at the end).",
    "",
    "Note this repo's rule that only Rubén marks a task Done, and that on this repo Done means",
    "flashed *and* driven. Several entries below record work that is finished and pushed but still",
    "awaiting that confirmation; each says so in its own text.",
    "",
    "## Closed items from the board",
    "",
]
last_heading = None
for h, b in archived_from_board:
    if h != last_heading:
        if h:
            out += ["", h.replace("## ", "### ").replace("#### ", "### "), ""]
        last_heading = h
    out += b + [""]

out += ["", "## Previously under the board's `## Done` heading", ""] + archived_from_done + [""]

with open("/Users/rubenayla/repos/hardware/kart-medulla/tasks/done-archive.md", "w") as f:
    f.write("\n".join(out).rstrip() + "\n")

# --- rewrite the board -----------------------------------------------------
board = kept
if any(re.match(r"^- \[ \]", l) for l in reopened):
    board += [
        "",
        "### Docs across kart-medulla, dv-hardware and kart-docs contradict the code and each other",
        "",
        "Found 2026-07-30 during a three-repo audit. These sat under the board's `## Done` heading while",
        "still open — the closed half of the section is now in `tasks/done-archive.md`.",
        "",
    ] + reopened

with open(TASKS, "w") as f:
    f.write("\n".join(board).rstrip() + "\n")
print("\nwritten.")
