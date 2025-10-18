#!/usr/bin/env python3
"""Summarize motion and GPS report files into CSV format."""

import argparse
import warnings
import sys
from pathlib import Path

import pandas as pd

INDEX_NAME = 'id'


def parse_report_content(content):
    """Parse entire report content into a dictionary."""
    return {
        line.split(':', 1)[0].strip(): line.split(':', 1)[1].strip()
        for line in content.split('\n')
        if ':' in line
    }


def parse_report(file_path):
    """Parse a single report file and return data dict."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        return parse_report_content(content)
    except (IOError, OSError) as e:
        print(f"Warning: Could not read {file_path}: {e}", file=sys.stderr)
        return None


def parse_reports(output_dir, suffix):
    """Parse report files and extract data."""
    # Find all report files with given suffix
    files = sorted(output_dir.glob(f"**/*{suffix}"))

    def get_name(file_path):
        return str(file_path.relative_to(output_dir)).replace(suffix, '').replace('\\', '/')

    dicts = [{INDEX_NAME: get_name(file_path), **parse_report(file_path)} for file_path in files]

    # Validate fields and warn about mismatches between files
    expected_fields = set(dicts[0].keys())
    for dict in dicts:
        actual_fields = set(dict.keys())
        if actual_fields != expected_fields:
            name = dict[INDEX_NAME]
            warnings.warn(
                f"Warning: {name} has unexpected fields: {', '.join(actual_fields)}, expected: {', '.join(expected_fields)}"
            )

    return pd.DataFrame(dicts).set_index(INDEX_NAME)


def create_unified_summary(motion_df, gps_df, correctness_expr):
    """Create unified summary combining motion and GPS data with classification."""
    if motion_df.empty and gps_df.empty:
        return pd.DataFrame()

    # Merge DataFrames on index
    unified_df = motion_df.join(gps_df, how='outer', rsuffix='_gps')

    def classify_row(row):
        try:
            return eval(correctness_expr, globals(), row.to_dict())
        except (ValueError, TypeError, NameError, SyntaxError):
            return None

    unified_df['correctness'] = unified_df.apply(classify_row, axis=1)

    # Reset index to make video identifier a column
    unified_df = unified_df.reset_index()

    return unified_df


def write_csv(filename, data):
    """Write DataFrame to CSV file."""
    try:
        data.to_csv(filename, index=False, quoting=1, encoding='utf-8')
        return True
    except (IOError, OSError) as e:
        print(f"Error: Could not write {filename}: {e}", file=sys.stderr)
        return False


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Create CSV summary files from motion and GPS report files")
    parser.add_argument("output_dir", type=Path, help="Directory containing the report files")
    parser.add_argument(
        "--unified",
        "-u",
        action="store_true",
        help="Create unified summary combining motion and GPS data with correctness classification")
    parser.add_argument(
        "--correctness-expr",
        default="correlation > 0.45 and course_range_deg > 45 and abs(offset) < 4",
        help=
        "Expression for correctness classification (default: correlation > 0.45 and course_range_deg > 45 and abs(offset) < 4)"
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    output_dir = args.output_dir

    # Validate that output directory exists and is a directory
    if not output_dir.exists() or not output_dir.is_dir():
        print(f"Error: '{output_dir}' is not a valid directory", file=sys.stderr)
        sys.exit(1)

    # Print summary header
    print(f"Summarizing reports from: {output_dir}")

    # Process motion and GPS reports - Dynamically discover fields
    kinds = ["motion", "gps"]

    kind_to_data = {}
    for kind in kinds:
        df = parse_reports(output_dir, suffix=f"_{kind}.txt")
        kind_to_data[kind] = df
        # Get fields from dataframe columns (excluding index)
        fields = [col for col in df.columns if col != df.index.name]
        print(f"Found {len(fields)} fields in {kind} reports: {', '.join(fields)}")
    kind_to_data["unified"] = create_unified_summary(kind_to_data["motion"], kind_to_data["gps"],
                                                     args.correctness_expr)

    created_files = []
    total_videos = 0
    for kind, data in kind_to_data.items():
        if not data.empty:
            filename = f"{kind}_summary.csv"
            write_csv(output_dir / filename, data)
            created_files.append(filename)
            total_videos = max(total_videos, len(data))  # Use the largest count as total videos
        else:
            print(f"No {kind} report files found")

    # Print summary footer with more details
    if created_files:
        print(f"\nSummary: Processed {total_videos} videos, created {len(created_files)} CSV files in {output_dir}:")
        for filename in created_files:
            print(f"  - {output_dir / filename}")
    else:
        print(f"\nNo CSV summary files created in: {output_dir}")


if __name__ == "__main__":
    main()
