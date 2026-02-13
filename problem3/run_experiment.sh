DOMAIN_FILE="domain.hddl"
PROBLEM_FILE="problem.hddl"
LOGS_DIR="logs"
PLAN_DIR="$LOGS_DIR/panda.plan"

rm -rf "$LOGS_DIR"
mkdir -p "$LOGS_DIR"

echo -e "Run PANDA planner for hierarchical task network"

# Run planner
java -jar PANDA.jar -parser hddl "$DOMAIN_FILE" "$PROBLEM_FILE"  > "$LOGS_DIR/panda.log" 2>&1

# Extract plan
grep -E '^[0-9]+:' "$LOGS_DIR/panda.log" > "$PLAN_DIR"

echo -e "Plan obtained"