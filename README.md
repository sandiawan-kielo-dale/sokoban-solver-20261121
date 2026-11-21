# Sokoban solver — push-optimal room-decomposed A*

Entry for the make.fun competition **"Build the best Sokoban solver, 21 November 2026"**
(`sokoban-solver-01-run-20261121`, deadline Mon 23 Nov 2026 03:00 UTC, $100 prize pool).

> Note on badges: the kit's `RULES.txt` contains no badge/shield markdown
> (searched the whole kit: no `badge`, `shield`, or `![...](...)` lines), so
> there are no kit badges to reproduce verbatim here. Competition page:
> https://make.fun — competition `sokoban-solver-01-run-20261121`.

## Results on the practice set

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

**Total practice score: 1,000,000 / 1,000,000** — push-optimal on every
instance. Whole 5-instance run takes ~0.4 s (limit 60 s), and two runs
write byte-identical output.

## How it works

Every generated level is a one-cell-wide hallway (the row the player
starts on) with 1–4 small rooms opening off it through a single door
each. A box pushed out of a room through its door can never come back,
so each room is independent and the level optimum is the sum of the
room optima. The solver therefore:

1. **Finds the hallway row** (the row containing `@`) and flood-fills
   each room off it, recording its door and hallway entry cell.
2. **Solves each room independently** with A\* over pushes. A state is
   the player cell plus the sorted box cells; successors are pushes,
   with the pushing square reached by a player BFS that treats boxes
   as walls. The heuristic is the minimum-cost assignment of boxes to
   goals (exact brute-force permutation for ≤ 5 boxes) over
   precomputed push distances that ignore other boxes — admissible,
   since each push moves one box one square.
3. **Prunes deadlocks**: "live" cells from reverse pull-reachability
   out of the goals, plus a frozen-2×2-block check (any 2×2 square of
   boxes/walls holding a box that is not on a goal can never move).
   Only the box that just moved can newly die or freeze, so those
   checks are incremental. A transposition table keeps the best push
   count per state; states pack into a single number, so the table
   and the heap stay cheap.
4. **Stitches rooms together** with shortest player walks along the
   hallway and verifies the whole move string by replaying it.
   A whole-level push search is kept as a fallback in case a level
   does not match the hallway/room shape.

If A\* exhausts its fixed expansion budget, greedy best-first and
then uniform-cost search are tried with their own fixed budgets.
Everything is budgeted by deterministic node counts only — a
per-instance budget of 200,000 expansions, so one hard instance can
never starve the others — with no clock reads and no randomness, so
two executions write byte-identical output.

## Running it

```sh
node main.ts   # reads $MAKEFUN_INPUT_DIR (default /input),
               # writes $MAKEFUN_OUTPUT_DIR/<id>/solution.txt (default /output)
```

Node 22+ (TypeScript types stripped natively; the code is plain
JavaScript in a `.ts` file and uses only `node:fs` / `node:path`).
No dependencies, no network, no files besides `main.ts`.

## Files

- `main.ts` — the whole solver (entrypoint, run as `node /bot/main.ts`)
- `README.md` — this file
- `LICENSE` — MIT
- `.makefun/owner.json` — claim file for wallet
  `2ujTeJdpkvEBdVoq6kKmhXHGgUV4KHSP3usfsQFfrYkf`
