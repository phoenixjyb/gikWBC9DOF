# Configuration Utilities

This directory contains utility scripts for managing project configuration.

## YAML to JSON Converter

### Overview

The `yaml_to_json.py` script converts YAML configuration files to JSON format for MATLAB compatibility. This is needed because:

- **YAML is the source of truth**: Easy to read, supports comments, used by future C++ code
- **MATLAB uses JSON**: `jsondecode()` is built-in and reliable across MATLAB versions
- **Python bridges the gap**: PyYAML + json provides robust conversion

### Workflow

```
config/pipeline_profiles.yaml  (source of truth - edit this)
          ↓
    python3 utils/yaml_to_json.py
          ↓
config/pipeline_profiles.json  (auto-generated - don't edit)
          ↓
    gik9dof.loadPipelineProfile('default')  (MATLAB reads JSON)
```

### Usage

**Convert default configuration:**
```bash
python3 utils/yaml_to_json.py
```

**Convert specific file:**
```bash
python3 utils/yaml_to_json.py path/to/config.yaml
```

**Specify output path:**
```bash
python3 utils/yaml_to_json.py input.yaml -o output.json
```

### When to Run

Run the converter whenever you edit `config/pipeline_profiles.yaml`:

1. Edit YAML file (e.g., change `yaw_corridor_deg: 15` to `yaw_corridor_deg: 20`)
2. Run `python3 utils/yaml_to_json.py`
3. The JSON file is updated automatically
4. MATLAB will use the new values next time it loads the profile

### Setup

**Install dependencies:**
```bash
# Using system Python
pip3 install pyyaml

# Using project virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install pyyaml
```

### Integration with Build System

For future automation, consider adding to your build workflow:

**Makefile example:**
```makefile
.PHONY: config
config:
	python3 utils/yaml_to_json.py
```

**Pre-commit hook example:**
```bash
#!/bin/bash
# .git/hooks/pre-commit
if git diff --cached --name-only | grep -q "config/pipeline_profiles.yaml"; then
    echo "YAML config changed, regenerating JSON..."
    python3 utils/yaml_to_json.py
    git add config/pipeline_profiles.json
fi
```

### Troubleshooting

**"ModuleNotFoundError: No module named 'yaml'"**
- Install PyYAML: `pip3 install pyyaml`

**"Configuration file not found"**
- Run from project root: `cd /path/to/gikWBC9DOF && python3 utils/yaml_to_json.py`

**"JSON file not found" error in MATLAB**
- Run the converter first: `python3 utils/yaml_to_json.py`
- Check that `config/pipeline_profiles.json` exists

## Future Utilities

This directory will contain additional utilities:

- **C++ code generation**: Generate header files from YAML config
- **Parameter validation**: Check consistency across profiles
- **Profile visualization**: Compare parameters across profiles
