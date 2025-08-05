#!/bin/bash

# Script to run move_lightbar program multiple times
# Usage: ./run_move_lightbar.sh [iterations] [log_level]

# Default values
DEFAULT_ITERATIONS=5
DEFAULT_LOG_LEVEL=2  # INFO level

# Get parameters
ITERATIONS=${1:-$DEFAULT_ITERATIONS}
LOG_LEVEL=${2:-$DEFAULT_LOG_LEVEL}

# Validate parameters
if ! [[ "$ITERATIONS" =~ ^[0-9]+$ ]] || [ "$ITERATIONS" -lt 1 ]; then
    echo "Error: Iterations must be a positive integer"
    echo "Usage: $0 [iterations] [log_level]"
    echo "  iterations: Number of times to run move_lightbar (default: $DEFAULT_ITERATIONS)"
    echo "  log_level: Log level 0-5 (default: $DEFAULT_LOG_LEVEL)"
    exit 1
fi

if ! [[ "$LOG_LEVEL" =~ ^[0-5]$ ]]; then
    echo "Error: Log level must be 0-5"
    echo "Usage: $0 [iterations] [log_level]"
    echo "  iterations: Number of times to run move_lightbar (default: $DEFAULT_ITERATIONS)"
    echo "  log_level: Log level 0-5 (default: $DEFAULT_LOG_LEVEL)"
    exit 1
fi

# Check if move_lightbar executable exists
if [ ! -f "./tests/move_lightbar" ]; then
    echo "Error: move_lightbar executable not found at ./tests/move_lightbar"
    echo "Please build the program first with: make test_move_lightbar"
    exit 1
fi

echo "Starting move_lightbar test run"
echo "================================"
echo "Iterations: $ITERATIONS"
echo "Log level: $LOG_LEVEL"
echo "Program: ./tests/move_lightbar"
echo ""

# Run the program multiple times
for ((i=1; i<=$ITERATIONS; i++)); do
    echo "=== Run $i of $ITERATIONS ==="
    echo "Timestamp: $(date)"
    echo ""
    
    # Run the program
    ./tests/move_lightbar "$LOG_LEVEL"
    
    EXIT_CODE=$?
    
    echo ""
    if [ $EXIT_CODE -eq 0 ]; then
        echo "✓ Run $i completed successfully"
    else
        echo "✗ Run $i failed with exit code $EXIT_CODE"
    fi
    
    echo ""
    
    # Add a small delay between runs (optional)
    if [ $i -lt $ITERATIONS ]; then
        echo "Waiting 2 seconds before next run..."
        sleep 2
        echo ""
    fi
done

echo "=== Test Run Complete ==="
echo "Total runs: $ITERATIONS" 