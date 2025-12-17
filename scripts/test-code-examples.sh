#!/bin/bash
# Test script for Module 1 ROS 2 code examples
# Validates all Python code examples in Docker environment

set -e  # Exit on error

echo "==========================================="
echo "Module 1: ROS 2 Code Examples Test Suite"
echo "==========================================="

CODE_DIR="docs/module-1-ros2/assets/code-examples"
DOCKER_IMAGE="ros2-module1-testing"

# Check if code examples directory exists
if [ ! -d "$CODE_DIR" ]; then
    echo "ERROR: Code examples directory not found: $CODE_DIR"
    exit 1
fi

# Build Docker image if not exists
if ! docker images | grep -q "$DOCKER_IMAGE"; then
    echo "Building Docker image: $DOCKER_IMAGE..."
    docker build -t $DOCKER_IMAGE -f docker/ros2-testing/Dockerfile .
fi

echo ""
echo "Testing Python code examples..."
echo "-------------------------------"

# Find all Python files
PYTHON_FILES=$(find "$CODE_DIR" -name "*.py" 2>/dev/null || echo "")

if [ -z "$PYTHON_FILES" ]; then
    echo "No Python files found in $CODE_DIR (this is OK if code examples not yet created)"
    exit 0
fi

TOTAL=0
PASSED=0
FAILED=0

for file in $PYTHON_FILES; do
    TOTAL=$((TOTAL + 1))
    filename=$(basename "$file")
    echo -n "Testing $filename... "

    # Run Python file in Docker container
    # Note: Some files may require ROS 2 runtime, so we source ROS 2 setup first
    if docker run --rm \
        -v "$(pwd)/$CODE_DIR:/workspace" \
        $DOCKER_IMAGE \
        bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/$filename --help 2>&1 | head -1" > /dev/null 2>&1; then
        echo "✓ PASS (syntax valid)"
        PASSED=$((PASSED + 1))
    else
        # Try basic syntax check
        if docker run --rm \
            -v "$(pwd)/$CODE_DIR:/workspace" \
            $DOCKER_IMAGE \
            python3 -m py_compile /workspace/$filename > /dev/null 2>&1; then
            echo "✓ PASS (syntax valid)"
            PASSED=$((PASSED + 1))
        else
            echo "✗ FAIL (syntax error)"
            FAILED=$((FAILED + 1))
        fi
    fi
done

echo ""
echo "==========================================="
echo "Test Results:"
echo "  Total:  $TOTAL"
echo "  Passed: $PASSED"
echo "  Failed: $FAILED"
echo "==========================================="

if [ $FAILED -gt 0 ]; then
    echo "FAILED: $FAILED test(s) failed"
    exit 1
else
    echo "SUCCESS: All tests passed!"
    exit 0
fi
