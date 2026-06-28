# CAI104 — Assessment 3 Requirements Checklist

Mapping of every requirement from the **Assessment 3 brief + rubric** to how
this submission addresses it. (`[x]` = met, `[~]` = not applicable / note.)

## A. Task requirements (from the brief)

- [x] **Program an AI agent to solve the case study** — `path_planner.py`
      implements a robot path planner.
- [x] **Formulate & frame the real-world problem** — Section 1 of the document
      formulates the maze as a state-space search (state, initial state, goal
      test, successor function, step cost).
- [x] **Choose a suitable AI technique** — A* search selected and justified in
      Section 2 (vs BFS, DFS, Greedy).
- [x] **Develop the technique in a modern programming language** — Python 3.
- [x] **Discuss applications & ethics** — covered in the reflective report
      ("What I Am Not Sure About" discusses real-world deployment & ethics).
- [x] **Mark the shortest path with `2`** (not 1/0/-1/9) — output marks the
      path with `2`; start `-1` and goal `9` preserved.

## B. Pseudocode requirements

- [x] **Clear and detailed** — full A* pseudocode in Section 3.
- [x] **Structured & best practices** — modular (main loop, Manhattan,
      Reconstruct).
- [x] **Enough comments** — inline `//` comments throughout.
- [x] **Flowchart to complement** — flowchart included in Section 4.
- [x] **Pseudocode NOT in word count** — kept separate; explicitly noted.

## C. Reflective report (2000 words ±10%)

- [x] **Overview**
- [x] **What went right**
- [x] **What went wrong**
- [x] **What you are not sure about**
- [x] **Conclusion**
- [x] **Word count = 1,853** (within 1,800–2,200 range)

## D. Rubric attributes

- [x] **Knowledge representation (40%)** — maze as a 2D array + state-space
      formulation (standard AI representation).
- [x] **Search algorithm (40%)** — A* with admissible Manhattan heuristic;
      verified efficient (148 nodes) and optimal.
- [x] **Reflective essay (20%)** — all required sections, with elaboration and
      justification.

## E. Naming & submission

- [x] **Source code folder: `Source – Student Name`** — `Source - Chelo Laffin`
      (uses a hyphen; rename to en-dash "–" if your facilitator is strict).
- [~] **Solution: `YourGameName.sln`** — NOT APPLICABLE. The brief hint implies
      a Visual Studio (C#) project, but Python was chosen, so there is no
      `.sln`. Convert to C# only if your facilitator assigned that.
- [x] **Submit MS Word or PDF with pseudocode + report** —
      `CAI104_Assessment3_Laffin_Chelo.pdf`.

## F. Independent verification (evidence)

- [x] Maze matches the brief exactly (12×24 = 288 cells, cell-by-cell check).
- [x] Start = (2, 2), Goal = (4, 20) — matches `-1` and `9` positions.
- [x] Path is contiguous, in-bounds, wall-free, runs Start → Goal.
- [x] **Optimality confirmed** by an independent BFS: A* = 54 moves,
      BFS = 54 moves (identical → shortest path).

---
*Note: this checklist is a working/tracking aid, not a submission deliverable.*
