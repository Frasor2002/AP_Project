DOMAIN_FILE="domain.hddl"
PROBLEM_FILE="problem.hddl"

echo -e "Run PANDA planner for hierarchical task network"

java -jar PANDA.jar -parser hddl domain.hddl problem.hddl

echo -e "Plan obtained"