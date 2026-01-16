#!/usr/bin/env python3
"""
Aggregate Coverage Report Generator

Parses individual device JSON coverage reports and generates aggregate statistics.
"""

import json
import sys
import os
from datetime import datetime
from pathlib import Path


def extract_device_name(json_path):
    """Extract device name from path like devices/LP8772x-Q1/test/coverage/*.json"""
    parts = Path(json_path).parts
    try:
        device_idx = parts.index('devices')
        return parts[device_idx + 1]
    except (ValueError, IndexError):
        # Fallback to filename parsing
        return Path(json_path).stem.replace('_coverage', '')


def load_coverage_report(json_path):
    """Load and parse a JSON coverage report"""
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)

        device_name = extract_device_name(json_path)

        return {
            'name': device_name,
            'coverage': data.get('total_coverage', 0.0),
            'total_lines': data.get('total_lines', 0),
            'covered_lines': data.get('covered_lines', 0),
            'uncovered_lines': data.get('uncovered_lines', 0),
            'report_file': json_path,
            'files': data.get('files', [])
        }
    except Exception as e:
        print(f"Error loading {json_path}: {e}", file=sys.stderr)
        return None


def generate_text_report(aggregate_data, devices):
    """Generate formatted text report"""
    lines = []
    lines.append("=" * 80)
    lines.append("AGGREGATE COVERAGE REPORT - ALL DEVICES")
    lines.append("=" * 80)
    lines.append("")
    lines.append(f"{'Device':<24} {'Total':>8} {'Covered':>8} {'Uncovered':>10} {'Coverage':>12}")
    lines.append("-" * 80)

    for device in devices:
        lines.append(
            f"{device['name']:<24} "
            f"{device['total_lines']:>8} "
            f"{device['covered_lines']:>8} "
            f"{device['uncovered_lines']:>10} "
            f"{device['coverage']:>11.2f}%"
        )

    lines.append("-" * 80)
    lines.append(
        f"{'TOTAL':<24} "
        f"{aggregate_data['total_lines']:>8} "
        f"{aggregate_data['covered_lines']:>8} "
        f"{aggregate_data['uncovered_lines']:>10} "
        f"{aggregate_data['total_coverage']:>11.2f}%"
    )
    lines.append("=" * 80)

    # Add per-device file-level reports
    for device in devices:
        device_report = generate_device_file_report(
            device['name'],
            device.get('files', [])
        )
        lines.append(device_report)

    return '\n'.join(lines)


def generate_device_file_report(device_name, files_data):
    """
    Generate file-by-file coverage report for a single device.

    Args:
        device_name: Name of the device (e.g., "TPS65386x-Q1")
        files_data: List of file coverage dictionaries

    Returns:
        Formatted text report as string
    """
    lines = []
    lines.append("")
    lines.append("=" * 80)
    lines.append(f"COVERAGE REPORT ({device_name})")
    lines.append("=" * 80)
    lines.append("")

    if not files_data:
        lines.append("No file-level coverage data available")
        lines.append("=" * 80)
        return '\n'.join(lines)

    # Table header - match generate_coverage_report.py format
    lines.append(f"{'File':<30} {'Total':>10} {'Covered':>10} {'Uncovered':>12} {'Coverage':>10}")
    lines.append("-" * 80)

    # Sort files by coverage percentage (lowest first to highlight gaps)
    sorted_files = sorted(files_data, key=lambda x: x.get('coverage_percent', 0))

    # Calculate device totals for summary row
    total_lines = sum(f.get('total_lines', 0) for f in files_data)
    covered_lines = sum(f.get('covered_lines', 0) for f in files_data)
    uncovered_lines = sum(f.get('uncovered_lines', 0) for f in files_data)

    # File rows
    for file_data in sorted_files:
        name = file_data.get('name', 'unknown')
        total = file_data.get('total_lines', 0)
        covered = file_data.get('covered_lines', 0)
        uncovered = file_data.get('uncovered_lines', 0)
        percent = file_data.get('coverage_percent', 0.0)

        lines.append(f"{name:<30} {total:>10} {covered:>10} {uncovered:>12} {percent:>9.2f}%")

    # Summary row
    lines.append("-" * 80)
    total_percent = (covered_lines / total_lines * 100) if total_lines > 0 else 0.0
    lines.append(f"{'TOTAL':<30} {total_lines:>10} {covered_lines:>10} "
                 f"{uncovered_lines:>12} {total_percent:>9.2f}%")
    lines.append("=" * 80)

    return '\n'.join(lines)


def generate_json_report(aggregate_data, devices):
    """Generate JSON format report with device and file-level data"""
    # Create devices list with file data included
    devices_with_files = []
    for device in devices:
        device_dict = {
            'name': device['name'],
            'coverage': device['coverage'],
            'total_lines': device['total_lines'],
            'covered_lines': device['covered_lines'],
            'uncovered_lines': device['uncovered_lines'],
            'report_file': device['report_file']
        }
        # Include file-level data if available
        if 'files' in device and device['files']:
            device_dict['files'] = device['files']
        devices_with_files.append(device_dict)

    return json.dumps({
        'timestamp': aggregate_data['timestamp'],
        'total_coverage': aggregate_data['total_coverage'],
        'total_lines': aggregate_data['total_lines'],
        'covered_lines': aggregate_data['covered_lines'],
        'uncovered_lines': aggregate_data['uncovered_lines'],
        'devices': devices_with_files
    }, indent=2)


def main():
    if len(sys.argv) < 2:
        print("Usage: aggregate_coverage_reports.py <json_file1> [json_file2 ...]", file=sys.stderr)
        print("", file=sys.stderr)
        print("Parses individual device JSON coverage reports and generates aggregate statistics.", file=sys.stderr)
        sys.exit(1)

    # Load all device reports
    devices = []
    for json_path in sys.argv[1:]:
        if not os.path.exists(json_path):
            print(f"Warning: File not found: {json_path}", file=sys.stderr)
            continue

        device_data = load_coverage_report(json_path)
        if device_data:
            devices.append(device_data)

    if not devices:
        print("ERROR: No valid coverage reports found!", file=sys.stderr)
        sys.exit(1)

    # Sort devices by name for consistent output
    devices.sort(key=lambda d: d['name'])

    # Calculate aggregate statistics
    total_lines = sum(d['total_lines'] for d in devices)
    covered_lines = sum(d['covered_lines'] for d in devices)
    uncovered_lines = sum(d['uncovered_lines'] for d in devices)
    total_coverage = (covered_lines / total_lines * 100) if total_lines > 0 else 0.0

    aggregate_data = {
        'timestamp': datetime.now().isoformat(),
        'total_coverage': total_coverage,
        'total_lines': total_lines,
        'covered_lines': covered_lines,
        'uncovered_lines': uncovered_lines
    }

    # Create output directory
    output_dir = Path('coverage')
    output_dir.mkdir(exist_ok=True)

    # Generate timestamp for filenames
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Generate and save text report
    text_report = generate_text_report(aggregate_data, devices)
    text_file = output_dir / f'{timestamp}_aggregate_coverage.txt'
    with open(text_file, 'w') as f:
        f.write(text_report)
    print(f"Text report saved: {text_file}")

    # Generate and save JSON report
    json_report = generate_json_report(aggregate_data, devices)
    json_file = output_dir / f'{timestamp}_aggregate_coverage.json'
    with open(json_file, 'w') as f:
        f.write(json_report)
    print(f"JSON report saved: {json_file}")

    # Print summary to stdout
    print("")
    print(text_report)


if __name__ == '__main__':
    main()
