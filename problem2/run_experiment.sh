#!/bin/bash

# Files
DOMAIN_FILE="domain.pddl"
PROBLEM_FILE_SMALL="problem_small.pddl"
PROBLEM_FILE_MEDIUM="problem_medium.pddl" # 5 artifacts per-type
PROBLEM_FILE_BIG="problem_big.pddl" # 10 artifacts per-type

LOGS_DIR="logs"
COMPARISON_DIR="logs/comparison"
SCALABILITY_DIR="logs/scalability"
PLAN_DIR="$LOGS_DIR/plan"

rm -rf "$LOGS_DIR"
mkdir -p "$LOGS_DIR"
mkdir -p "$COMPARISON_DIR"
mkdir -p "$SCALABILITY_DIR"
mkdir -p "$PLAN_DIR"


echo -e "Comparing planners"

# First we run all aliases

# lama
planutils run downward "--alias lama --plan-file $PLAN_DIR/lama.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/lama.log" 2>&1

# lama-first
planutils run downward "--alias lama-first --plan-file $PLAN_DIR/lama_first.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/lama_first.log" 2>&1

# seq-opt-bjolp
planutils run downward "--alias seq-opt-bjolp --plan-file $PLAN_DIR/bjolp.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/bjolp.log" 2>&1

# seq-opt-fdss-1
planutils run downward -- "--alias seq-opt-fdss-1 --overall-time-limit 300 --plan-file $PLAN_DIR/opt_fdss_1.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/opt_fdss_1.log" 2>&1

# seq-opt-fdss-2
planutils run downward -- "--alias seq-opt-fdss-2 --overall-time-limit 300 --plan-file $PLAN_DIR/opt_fdss_2.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/opt_fdss_2.log" 2>&1

# seq-opt-lmcut
planutils run downward "--alias seq-opt-lmcut --plan-file $PLAN_DIR/lmcut.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/lmcut.log" 2>&1

# seq-opt-merge-and-shrink
planutils run downward -- "--alias seq-opt-merge-and-shrink --overall-time-limit 300 --plan-file $PLAN_DIR/merge_and_shrink.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/merge_and_shrink.log" 2>&1

# seq-sat-fd-autotune-1
planutils run downward "--alias seq-sat-fd-autotune-1 --plan-file $PLAN_DIR/autotune_1.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/autotune_1.log" 2>&1

# seq-sat-fd-autotune-2
planutils run downward "--alias seq-sat-fd-autotune-2 --plan-file $PLAN_DIR/autotune_2.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/autotune_2.log" 2>&1

# seq-sat-fdss-1
planutils run downward -- "--alias seq-sat-fdss-1 --overall-time-limit 300 --plan-file $PLAN_DIR/sat_fdss_1.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/sat_fdss_1.log" 2>&1

# seq-sat-fdss-2
planutils run downward -- "--alias seq-sat-fdss-2 --overall-time-limit 300 --plan-file $PLAN_DIR/sat_fdss_2.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/sat_fdss_2.log" 2>&1

# seq-sat-fdss-2014
planutils run downward -- "--alias seq-sat-fdss-2014 --overall-time-limit 300 --plan-file $PLAN_DIR/fdss_2014.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/fdss_2014.log" 2>&1

# seq-sat-fdss-2018
planutils run downward -- "--alias seq-sat-fdss-2018 --overall-time-limit 300 --plan-file $PLAN_DIR/fdss_2018.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/fdss_2018.log" 2>&1

# seq-sat-fdss-2023
planutils run downward -- "--alias seq-sat-fdss-2023 --overall-time-limit 300 --plan-file $PLAN_DIR/fdss_2023.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/fdss_2023.log" 2>&1

# seq-sat-lama2011
planutils run downward "--alias seq-sat-lama-2011 --plan-file $PLAN_DIR/lama_2011.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$COMPARISON_DIR/lama_2011.log" 2>&1


# Search options

# astar blind
planutils run downward "--plan-file $PLAN_DIR/astar_blind.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(blind())" > "$COMPARISON_DIR/astar_blind.log" 2>&1

# astar ff
planutils run downward "--plan-file $PLAN_DIR/astar_ff.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(ff())" > "$COMPARISON_DIR/astar_ff.log" 2>&1

# astar add
planutils run downward "--plan-file $PLAN_DIR/astar_add.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(add())" > "$COMPARISON_DIR/astar_add.log" 2>&1

# astar lmcut
planutils run downward "--plan-file $PLAN_DIR/astar_lmcut.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(lmcut())" > "$COMPARISON_DIR/astar_lmcut.log" 2>&1

# astar cg
planutils run downward "--plan-file $PLAN_DIR/astar_cg.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(cg())" > "$COMPARISON_DIR/astar_cg.log" 2>&1

# astar cea
planutils run downward "--plan-file $PLAN_DIR/astar_cea.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(cea())" > "$COMPARISON_DIR/astar_cea.log" 2>&1

# astar hmax
planutils run downward "--plan-file $PLAN_DIR/astar_hmax.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search astar(hmax())" > "$COMPARISON_DIR/astar_hmax.log" 2>&1

# eager_greedy blind
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_blind.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([blind()])" > "$COMPARISON_DIR/eager_greedy_blind.log" 2>&1

# eager_greedy ff
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_ff.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([ff()])" > "$COMPARISON_DIR/eager_greedy_ff.log" 2>&1

# eager_greedy add
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_add.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
 "--search eager_greedy([add()])" > "$COMPARISON_DIR/eager_greedy_add.log" 2>&1

# eager_greedy lmcut
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_lmcut.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([lmcut()])" > "$COMPARISON_DIR/eager_greedy_lmcut.log" 2>&1

# eager_greedy cg
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_cg.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([cg()])" > "$COMPARISON_DIR/eager_greedy_cg.log" 2>&1

# eager_greedy cea
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_cea.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([cea()])" > "$COMPARISON_DIR/eager_greedy_cea.log" 2>&1

# eager_greedy hmax
planutils run downward "--plan-file $PLAN_DIR/eager_greedy_hmax.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search eager_greedy([hmax()])" > "$COMPARISON_DIR/eager_greedy_hmax.log" 2>&1

# lazy_wastar blind
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_blind.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([blind()],w=5)" > "$COMPARISON_DIR/lazy_wastar_blind.log" 2>&1

# lazy_wastar ff
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_ff.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([ff()],w=5)" > "$COMPARISON_DIR/lazy_wastar_ff.log" 2>&1

# lazy_wastar add
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_add.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([add()],w=5)" > "$COMPARISON_DIR/lazy_wastar_add.log" 2>&1

# lazy_wastar lmcut
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_lmcut.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([lmcut()],w=5)" > "$COMPARISON_DIR/lazy_wastar_lmcut.log" 2>&1

# lazy_wastar cg
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_cg.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([cg()],w=5)" > "$COMPARISON_DIR/lazy_wastar_cg.log" 2>&1

# lazy_wastar cea
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_cea.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([cea()],w=5)" > "$COMPARISON_DIR/lazy_wastar_cea.log" 2>&1

# lazy_wastar hmax
planutils run downward "--plan-file $PLAN_DIR/lazy_wastar_hmax.plan" "$DOMAIN_FILE" "$PROBLEM_FILE_SMALL" \
  "--search lazy_wastar([hmax()],w=5)" > "$COMPARISON_DIR/lazy_wastar_hmax.log" 2>&1


echo -e "Comparison completed. Logs in '$COMPARISON_DIR'"

echo -e "Scalability analysis"

planutils run downward "--alias lama-first --plan-file $PLAN_DIR/small.plan $DOMAIN_FILE $PROBLEM_FILE_SMALL" \
  > "$SCALABILITY_DIR/small.log" 2>&1

planutils run downward "--alias lama-first --plan-file $PLAN_DIR/medium.plan $DOMAIN_FILE $PROBLEM_FILE_MEDIUM" \
  > "$SCALABILITY_DIR/medium.log" 2>&1

planutils run downward "--alias lama-first --plan-file $PLAN_DIR/big.plan $DOMAIN_FILE $PROBLEM_FILE_BIG" \
  > "$SCALABILITY_DIR/big.log" 2>&1

  echo -e "Scalability analysis completed. Logs in $SCALABILITY_DIR'"
