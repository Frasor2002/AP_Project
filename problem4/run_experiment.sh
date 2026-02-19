DOMAIN_FILE="domain.pddl"
PROBLEM_FILE="problem.pddl"
LOGS_DIR="logs"
COMPARISON_DIR="$LOGS_DIR/comparison"
PLAN_DIR="$LOGS_DIR/plan"

rm -rf "$LOGS_DIR"
mkdir -p "$LOGS_DIR"
mkdir -p "$COMPARISON_DIR"
mkdir -p "$PLAN_DIR"

echo -e "Comparing temporal planners."

# Run planners for comparison

# Optic
planutils run optic -- -N $DOMAIN_FILE $PROBLEM_FILE -out "$PLAN_DIR/optic.plan" \
  > "$COMPARISON_DIR/optic.log" 2>&1
# Extract plan from log
awk '/Solution Found/{found=1} found' "$COMPARISON_DIR/optic.log" | 
    grep -E "^([0-9]|;)" > "$PLAN_DIR/optic.plan"

# Optic disable best-first search, use EHC
planutils run optic -- -N -b $DOMAIN_FILE $PROBLEM_FILE -out "$PLAN_DIR/optic_dis_bf.plan"\
  > "$COMPARISON_DIR/optic_dis_bf.log" 2>&1
# Extract plan from log
awk '/Solution Found/{found=1} found' "$COMPARISON_DIR/optic_dis_bf.log" | \
  grep -E "^([0-9]|;)" > "$PLAN_DIR/optic_dis_bf.plan"

# lpg-td planner
planutils run lpg-td -- -out "$PLAN_DIR/lpg_speed" -o "$DOMAIN_FILE" -f "$PROBLEM_FILE" -speed \
  > "$COMPARISON_DIR/lpg_speed.log" 2>&1

planutils run lpg-td --  -out "$PLAN_DIR/lpg_quality" -o "$DOMAIN_FILE" -f "$PROBLEM_FILE" -quality \
  > "$COMPARISON_DIR/lpg_quality.log" 2>&1


planutils run lpg-td -- -out "$PLAN_DIR/lpg_incremental" -o "$DOMAIN_FILE" -f "$PROBLEM_FILE"  -n 3 \
  > "$COMPARISON_DIR/lpg_incremental.log" 2>&1

echo -e "Comparison done, find logs in $LOGS_DIR."