-- ID: 170027939
-- CS-5099 MSC DISSERTATION
-- MOBILE ROBOT NAVIGATION
-- UNIVERSITY OF ST ANDREWS
-- Phase 1 Complete

DONE:
- Added implementations of Simulated Annealing, RRT and PotBug controllers for Webots
- Added two worlds for each algorithm. i.e. one with a Curve and the other, Freespace


ISSUES:
- Convergence issues:
PotBug implementation reduces the goal distance but does terminates abruptly before reaching the goal
Simulated Annealing seems to drive in reverse and has in consistencies
RRT keeps winding

All issues will be fixed

TO-DO:
- Implement simple metrics:
--convergence
--Time metrics + sub-goal times
--Cummulative path length
--Average Turn Angles
--Node Effienciency
--Rotation/Smoothness Effect

FINALLY:
- Refactor implementation
- Tidy up the code
- JavaDocs