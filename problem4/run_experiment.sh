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
planutils run optic -- -N $DOMAIN_FILE $PROBLEM_FILE -out "$PLAN_DIR/optic.plan"\
  > "$COMPARISON_DIR/optic.log" 2>&1
# Extract plan from log
awk '/Solution Found/{found=1} found' "$COMPARISON_DIR/optic.log" | \
    grep -E "^([0-9]|;)" > "$PLAN_DIR/optic.plan"

# TODO Add more planners

echo -e "Comparison done, find logs in $LOGS_DIR."