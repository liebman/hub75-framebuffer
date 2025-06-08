#!/bin/bash

# Test Coverage Script

set -e

echo "🔍 Running test coverage..."

# Clean previous coverage data
cargo llvm-cov clean

# Run coverage for blocking implementation (default features)
echo "📊 Testing..."
cargo llvm-cov --no-report test

# Generate coverage reports
echo "📋 Generating coverage reports..."

# Generate HTML report
cargo llvm-cov report --html --output-dir coverage/html

# Generate LCOV report (for CI/external tools)
cargo llvm-cov report --lcov --output-path coverage/lcov.info

# Generate summary to console
cargo llvm-cov report

echo "✅ Coverage analysis complete!"
echo "📁 HTML report: coverage/html/index.html"
echo "📁 LCOV report: coverage/lcov.info" 