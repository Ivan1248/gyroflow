#!/usr/bin/env python3
"""Summarize motion and GPS report files into CSV format."""

import argparse
import csv
import sys
from pathlib import Path
import pandas as pd


def parse_report_content(content):
    """Parse entire report content into a dictionary."""
    return {
        line.split(':', 1)[0].strip(): line.split(':', 1)[1].strip()
        for line in content.split('\n')
        if ':' in line
    }


def parse_report(file_path, output_dir, suffix, fields):
    """Parse a single report file and return data row."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Parse entire content into dictionary
        report_dict = parse_report_content(content)

        # Extract video name from file path relative to output directory
        rel_path = file_path.relative_to(output_dir)
        video_name = str(rel_path).replace(suffix, '').replace('\\', '/')

        # Filter requested fields from the report dictionary
        return [video_name] + [report_dict.get(field, "N/A") for field in fields]

    except (IOError, OSError) as e:
        print(f"Warning: Could not read {file_path}: {e}", file=sys.stderr)
        return None


def parse_reports(output_dir, suffix, fields):
    """Parse report files and extract data."""
    # Find all report files with given suffix
    files = sorted(output_dir.glob(f"**/*{suffix}"))
    if not files:
        return pd.DataFrame()

    rows = [
        row for file_path in files if (row := parse_report(file_path, output_dir, suffix, fields))
    ]

    if not rows:
        return pd.DataFrame()

    # Create DataFrame with video as index
    df = pd.DataFrame(rows, columns=['video'] + fields)
    df = df.set_index('video')
    return df


def create_unified_summary(motion_df, gps_df, correctness_expr):
    """Create unified summary combining motion and GPS data with classification."""
    if motion_df.empty and gps_df.empty:
        return pd.DataFrame()

    # Merge DataFrames on index (video names)
    unified_df = motion_df.join(gps_df, how='outer', rsuffix='_gps')

    def classify_row(row):
        try:
            # Evaluate the expression with the row as input
            return eval(correctness_expr, globals(), row.to_dict())
        except (ValueError, TypeError, NameError, SyntaxError):
            return None

    unified_df['correctness'] = unified_df.apply(classify_row, axis=1)

    # Reset index to make video a column
    unified_df = unified_df.reset_index()

    return unified_df


def write_csv(filename, header, data):
    """Write CSV data to file."""
    try:
        if isinstance(data, pd.DataFrame):
            # If data is a DataFrame, use pandas to_csv
            data.to_csv(filename, index=False, quoting=csv.QUOTE_ALL, encoding='utf-8')
        else:
            # If data is a list of lists, use csv writer
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f, quoting=csv.QUOTE_ALL)
                writer.writerow(header)
                writer.writerows(data)
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
        help="Expression for correctness classification (default: correlation > 0.45 and course_range_deg > 45 and abs(offset) < 4)")
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

    # Process motion and GPS reports - Headers are Single Source of Truth
    configs = [
        dict(kind="motion",
             header=["video", "stream", "avg_transl_dir", "is_moving_forward"],
             suffix="_motion.txt"),
        dict(kind="gps",
             header=["video", "offset", "similarity", "correlation", "course_range_deg"],
             suffix="_gps.txt")
    ]

    kind_to_data = {
        config["kind"]: parse_reports(output_dir, config["suffix"], config["header"][1:])
        for config in configs
    }
    kind_to_data["unified"] = create_unified_summary(kind_to_data["motion"], kind_to_data["gps"], args.correctness_expr)

    for kind, data in kind_to_data.items():
        if not data.empty:
            write_csv(output_dir / f"{kind}_summary.csv", None, data)
        else:
            print(f"No {kind} report files found")

    # Print summary footer
    print(f"\nCSV summary files created in: {output_dir}")


if __name__ == "__main__":
    main()
