rb# Poetry Environment Setup

The project now has a Poetry environment configured with the following dependencies:

## Installed Packages

### From Homebrew
- **graphviz** (14.1.2) - Graph visualization software
- **sip** (6.15.1) - Python binding generator
- **qt@5** (5.15.18) - Qt framework (dependency for PyQt5)
- **pyqt@5** (5.15.11_2) - Python bindings for Qt (system package)

### From Poetry (Python packages)
- **PyQt5** (5.15.11) - Python bindings for Qt framework
- **pydot** (4.0.1) - Python interface to Graphviz's Dot language
- **pygraphviz** (1.14) - Python interface to Graphviz (built with Homebrew paths)

## Installation Commands Used

```bash
# Initialize Poetry
poetry init --no-interaction --name ros2_hello_world

# Add PyQt5 and pydot
poetry add PyQt5 pydot

# Install pygraphviz with Homebrew graphviz paths
poetry run pip install pygraphviz \
  --config-settings="--global-option=build_ext" \
  --config-settings="--global-option=-I$(brew --prefix graphviz)/include/" \
  --config-settings="--global-option=-L$(brew --prefix graphviz)/lib/"
```

## Usage

To use the Poetry environment:

```bash
# Activate the environment
poetry shell

# Or run commands directly
poetry run python your_script.py
```

## Verification

All packages are successfully installed and importable:
- ✓ PyQt5 (Qt GUI framework)
- ✓ pydot (Graphviz Dot interface)
- ✓ pygraphviz (Graphviz Python interface)
