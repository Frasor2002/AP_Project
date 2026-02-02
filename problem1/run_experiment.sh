#!/bin/bash

# Files
DOMAIN_FILE="domain.pddl"
DEFAULT_PROBLEM="problem_small.pddl"
PROBLEM_FILE="${1:-$DEFAULT_PROBLEM}"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'


FD_ALIASES=(
  "lama-first"
  "seq-sat-lama-2011"
  "seq-opt-lmcut"
  "seq-opt-fdss-1"
  "seq-opt-bjolp"
  "seq-opt-merge-and-shrink"
)

FD_SEARCHES=(
  "eager_greedy([ff()])"
  "eager_greedy([add()])"
  "eager_greedy([cg()])"
  "astar(ff())"
  "lazy_wastar([ff()],w=5)"
)



PROBLEM_NAME=$(basename "$PROBLEM_FILE" .pddl)
LOG_DIR="logs/$PROBLEM_NAME"
mkdir -p "$LOG_DIR"

echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE} SOLVING: $PROBLEM_NAME ${NC}"
echo -e "${BLUE}========================================${NC}"

for ALIAS in "${FD_ALIASES[@]}"; do
  echo -ne "  > Running FD Alias: ${YELLOW}$ALIAS${NC} ... "
    
  planutils run downward "--alias $ALIAS $DOMAIN_FILE $PROBLEM_FILE" \
    > "$LOG_DIR/fd_alias_${ALIAS}.log" 2>&1
    
  EXIT_CODE=$?
  if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}DONE${NC}"
  elif [ $EXIT_CODE -eq 124 ]; then
    echo -e "${RED}TIMEOUT${NC}"
  else
    echo -e "${RED}FAILED${NC}"
  fi
done


for SEARCH in "${FD_SEARCHES[@]}"; do
  SAFE_NAME=$(echo "$SEARCH" | tr -c 'a-zA-Z0-9' '_')
  echo -ne "  > Running FD Search: ${YELLOW}${SEARCH:0:40}...${NC} "
    
  planutils run downward "$DOMAIN_FILE" "$PROBLEM_FILE" "--search $SEARCH" > "$LOG_DIR/fd_search_${SAFE_NAME}.log" 2>&1
    
  EXIT_CODE=$?
  if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}DONE${NC}"
  elif [ $EXIT_CODE -eq 124 ]; then
    echo -e "${RED}TIMEOUT${NC}"
  else
    echo -e "${RED}FAILED${NC}"
  fi
done



echo -e "\n${GREEN}Tests completed for $PROBLEM_FILE. Logs in '$LOG_DIR'${NC}"