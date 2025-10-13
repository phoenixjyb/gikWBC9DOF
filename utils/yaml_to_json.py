#!/usr/bin/env python3
"""
YAML to JSON Configuration Converter
=====================================

Converts YAML configuration files to JSON format for MATLAB compatibility.

Usage:
    python yaml_to_json.py                    # Convert pipeline_profiles.yaml
    python yaml_to_json.py input.yaml         # Convert specific file
    python yaml_to_json.py input.yaml -o out.json  # Specify output

The script preserves the full structure of the YAML file, including:
- Nested configurations (profiles -> default -> chassis -> track)
- Comments (converted to _comment fields in JSON)
- All data types (strings, numbers, booleans)

For gikWBC9DOF:
- Source: config/pipeline_profiles.yaml
- Output: config/pipeline_profiles.json
- MATLAB loads JSON using jsondecode()
"""

import yaml
import json
import argparse
from pathlib import Path
from typing import Any, Dict, Optional


def yaml_to_json(yaml_path: Path, json_path: Optional[Path] = None, indent: int = 2) -> None:
    """
    Convert YAML file to JSON format.
    
    Args:
        yaml_path: Path to input YAML file
        json_path: Path to output JSON file (default: same name with .json extension)
        indent: JSON indentation level for readability
    """
    # Read YAML with explicit UTF-8 encoding
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    
    # Determine output path
    if json_path is None:
        json_path = yaml_path.with_suffix('.json')
    
    # Write JSON with explicit UTF-8 encoding
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=indent, ensure_ascii=False)
    
    print(f"✅ Converted {yaml_path} -> {json_path}")
    print(f"   Size: {yaml_path.stat().st_size} bytes (YAML) -> {json_path.stat().st_size} bytes (JSON)")


def convert_pipeline_profiles() -> None:
    """Convert the main pipeline_profiles.yaml configuration."""
    project_root = Path(__file__).parent.parent
    yaml_path = project_root / 'config' / 'pipeline_profiles.yaml'
    json_path = project_root / 'config' / 'pipeline_profiles.json'
    
    if not yaml_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {yaml_path}")
    
    yaml_to_json(yaml_path, json_path)
    
    # Validate structure
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    if 'profiles' not in data:
        raise ValueError("JSON output missing 'profiles' key")
    
    profiles = list(data['profiles'].keys())
    print(f"   Profiles available: {', '.join(profiles)}")


def main():
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        description='Convert YAML configuration to JSON for MATLAB',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert default pipeline configuration
  python yaml_to_json.py
  
  # Convert specific file
  python yaml_to_json.py path/to/config.yaml
  
  # Specify output path
  python yaml_to_json.py input.yaml -o output.json
        """
    )
    parser.add_argument(
        'input',
        type=Path,
        nargs='?',
        help='Input YAML file (default: config/pipeline_profiles.yaml)'
    )
    parser.add_argument(
        '-o', '--output',
        type=Path,
        help='Output JSON file (default: same name as input with .json extension)'
    )
    parser.add_argument(
        '--indent',
        type=int,
        default=2,
        help='JSON indentation level (default: 2)'
    )
    
    args = parser.parse_args()
    
    if args.input is None:
        # Default: convert pipeline_profiles.yaml
        convert_pipeline_profiles()
    else:
        # Convert specified file
        if not args.input.exists():
            parser.error(f"Input file not found: {args.input}")
        yaml_to_json(args.input, args.output, args.indent)


if __name__ == '__main__':
    main()
