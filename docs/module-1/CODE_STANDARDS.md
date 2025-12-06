# Code Quality Standards for Module 1

## Python Code Standards

### Formatter: Black
```bash
# Install black
pip install black --break-system-packages

# Format a file
black example.py

# Check without modifying
black --check example.py
```

### Linter: Flake8
```bash
# Install flake8
pip install flake8 --break-system-packages

# Lint a file
flake8 example.py

# Configuration (create .flake8 in project root)
[flake8]
max-line-length = 88
extend-ignore = E203, W503
```

## PEP 8 Compliance
- All Python code examples must pass `black` formatter
- All code must pass `flake8` with no errors
- Line length: 88 characters (Black default)
- Inline comments: Every 3-5 lines of non-obvious code

## ROS 2 Specific
- Version: ROS 2 Humble
- Python: 3.10+
- Use rclpy (not rospy)
- Node naming: snake_case
- Package naming: snake_case