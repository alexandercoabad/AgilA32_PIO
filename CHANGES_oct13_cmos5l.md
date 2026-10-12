# 13 Oct 2026 - CMOS5L hardening results (documentation only)

No RTL, test or flow change in this commit. It records the result of gds run #90 (push `16b8345`, branch `try-density58`).

Files changed: `SUBMISSION.md`, `README.md`, `info.yaml` (comment only), `AgilA32_Project_Documentation.pdf` / `.docx` (also `docs/AgilA32_Project_Documentation.pdf`).

Result on the SG13 CMOS5L flow, 8x4 tiles, `PL_TARGET_DENSITY_PCT` 58:
- gds 4 h 34 min, precheck 4 h 47 min (9/9), gl_test 8 min (13/13), viewer 50 s; whole workflow 9 h 22 min.
- 38,504 cells excluding fill and tap, 41.8 % utilisation, wire length 2,478,845 um.

What was learned:
- 6x4 did not route (detailed routing did not converge).
- With density 60 % the precheck reported one Metal2 spacing violation (M2.b, about 1 nm at one corner); the flow is deterministic, so reruns repeat it. Density 58 % is clean. Density 55 % and a 41 ns clock hit the six-hour limit in gds.
- gl_test needs the cell library's `_udp.v` read before `_stdcell.v` (in `test/Makefile`).
- Earlier numbers labelled CMOS5L (51.45 %, 34,053 cells, runs #65 and #79) came from the SG13G2 flow at 6x4 and are now labelled that way.
