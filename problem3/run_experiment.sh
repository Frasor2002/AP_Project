DOMAIN_FILE="domain.hddl"
PROBLEM_FILE="problem.hddl"
LOGS_DIR="logs"
PLAN_DIR="$LOGS_DIR/plan"

rm -rf "$LOGS_DIR"
mkdir -p "$LOGS_DIR"
mkdir -p "$PLAN_DIR"

echo -e "Run PANDA planner for hierarchical task network"

# Run planner
# panda
start=$(date +%s.%N)
# this is because the files follow a different encoding for java -Dfile.encoding=ISO-8859-1
java -Dfile.encoding=ISO-8859-1 -jar PANDA.jar -parser hddl "$DOMAIN_FILE" "$PROBLEM_FILE"  > "$LOGS_DIR/panda.log" 2>&1
end=$(date +%s.%N)
runtime=$(awk "BEGIN {print $end - $start}")
sed -i "1iExecution Time: $runtime seconds" "$LOGS_DIR/panda.log"

# siadex
start=$(date +%s.%N)
singularity run siadex.sif "$DOMAIN_FILE" "$PROBLEM_FILE" > "$LOGS_DIR/siadex.log" 2>&1
end=$(date +%s.%N)
runtime=$(awk "BEGIN {print $end - $start}")
sed -i "1iExecution Time: $runtime seconds" "$LOGS_DIR/siadex.log"

# Extract plan

grep -E '^[0-9]+:' "$LOGS_DIR/panda.log" > "$PLAN_DIR/panda.plan"

sed -n '/^==>$/,/^root 0$/ {
  /^==>$/d                
  /^root 0$/d             
  s/^[0-9]\+[[:space:]]\+/(/ 
  s/$/)/                  
  p                       
}' "$LOGS_DIR/siadex.log" >> "$PLAN_DIR/siadex.plan"

echo -e "Plan obtained"