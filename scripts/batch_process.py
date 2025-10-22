#!/usr/bin/env python3
"""
Batch processing script for Gyroflow stabilization.

Processes subdirectories containing INSV and GPX files, aligning virtual camera
with forward motion direction and synchronizing GPS time.

Structure: input_root/
├── session1/
│   ├── video1.insv
│   └── track1.gpx
├── session2/
│   ├── video2.insv
│   └── track2.gpx
└── ...
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

# Default configuration
DEFAULT_PRESET = {
    "version": 2,
    "stabilization": {
        "fov": 3.222,
        "method": "Plain 3D",
        "smoothing_params": [{
            "name": "time_constant",
            "value": 1.0
        }],
        "horizon_lock_amount": 100.0,  # in percent
        "motion_direction_enabled": True,
        "motion_direction_params": [{
            "name": "flip_backward_dir",
            "value": False
        }]
    },
    "output": {
        "width": 1280,
        "height": 920
    }
}

# Default GPS synchronization settings
GPS_SETTINGS = {
    "sync_mode": "auto",
    "use_processed_motion": True,
    "speed_threshold": 1.5,
    "max_time_offset_s": 6.0
}


def process_directory(rec_dir: Path,
                      output_root_abs: Path,
                      gyroflow_cmd: str,
                      preset: dict,
                      keep_backward: bool = False,
                      no_render: bool = False) -> None:
    """Process a single recording directory."""
    rec_name = rec_dir.name
    out_dir = output_root_abs / rec_name
    out_dir.mkdir(parents=True, exist_ok=True)

    # Find INSV and GPX files
    vid_files = list(rec_dir.glob("*.insv"))
    gpx_files = list(rec_dir.glob("*.gpx"))

    if not vid_files:
        print(f"Skipping {rec_name}: no INSV file found")
        return
    if not gpx_files:
        print(f"Skipping {rec_name}: no GPX file found")
        return

    vid_file = vid_files[0]
    gpx_file = gpx_files[0]

    print(f"Processing: {rec_name}")
    vid_name = vid_file.stem
    out_stem = out_dir / vid_name

    fwd_stream = None

    # Process each stream (0 and 1)
    for stream in [0, 1]:
        cmd = [
            gyroflow_cmd,
            str(vid_file), "--stream",
            str(stream), "--preset",
            json.dumps(preset), "--overwrite", "-p",
            json.dumps({"output_path": f"{out_stem}_s{stream}.mp4"}), "--input-gpx",
            str(gpx_file), "--gps-settings",
            json.dumps(GPS_SETTINGS), "--export-gpx", f"{out_stem}_s{stream}.gpx",
            "--report-motion", f"{out_stem}_s{stream}_motion.txt", "--report-gps",
            f"{out_stem}_s{stream}_gps.txt"
        ]
        if no_render:
            cmd.append("--no-render")

        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=False, text=True)
        if result.returncode != 0:
            print(f"Error processing {rec_name}, stream {stream}: {result.stderr}")
            continue

        # Check if this stream has forward motion
        motion_report = out_stem.parent / f"{vid_name}_s{stream}_motion.txt"
        if motion_report.exists():
            if "is_looking_forward: 1" in motion_report.read_text():
                print(f"Video stream {stream} has forward motion")
                fwd_stream = stream if fwd_stream is None else fwd_stream
            else:
                print(f"Stream {stream} does not have forward motion")
                if not keep_backward:
                    print("Deleting backward-looking files except reports")
                    for ext in ["mp4", "gpx"][int(no_render):]:
                        (out_stem.parent / f"{vid_name}_s{stream}.{ext}").unlink(missing_ok=True)
                else:
                    print("Keeping backward-looking files (--keep-backward enabled)")
        else:
            print(f"Warning: Motion report not found for stream {stream}")

    # Rename files from the stream with forward motion (only if not keeping backward streams)
    if not keep_backward and fwd_stream is not None:
        try:
            file_mappings = [
                (f"{out_stem}_s{fwd_stream}.mp4", f"{out_stem}.mp4"),
                (f"{out_stem}_s{fwd_stream}.gpx", f"{out_stem}.gpx"),
                (f"{out_stem}_s{fwd_stream}_motion.txt", f"{out_stem}_motion.txt"),
                (f"{out_stem}_s{fwd_stream}_gps.txt", f"{out_stem}_gps.txt"),
            ][int(no_render):]
            for src, dst in file_mappings:
                shutil.move(src, dst)
            print(f"Renamed files from stream {fwd_stream} to final output")
        except FileNotFoundError as e:
            print(f"Warning: Could not rename some files: {e}")


EXAMPLE_USAGE = r"""
Examples:
  %(prog)s ./input ./output                                         # Process all videos
  %(prog)s ./input ./output --path-regex 'session.*/.*\.insv'     # Process all INSV files in session directories
  %(prog)s ./input ./output --path-regex 'session1/.*'             # Process videos in session1 directory
  %(prog)s ./input ./output --path-regex '.*video1\.insv'          # Process video1.insv in any directory
  %(prog)s ./input ./output --gyroflow-cmd ./gyroflow               # Use custom gyroflow path
  %(prog)s ./input ./output --keep-backward                         # Keep backward-looking streams
  %(prog)s ./input ./output --no-render                             # Process without rendering video
"""


def find_directories_to_process(input_root: Path, path_regex=None) -> set:
    """Find directories containing video files that match the path regex.

    Args:
        input_root: Root directory to search in
        path_regex: Optional compiled regex pattern to filter video paths

    Returns:
        Set of directory paths that contain matching video files
    """
    directories_to_process = set()
    for vid_file in input_root.rglob("*.insv"):
        # Get path relative to input_root
        rel_path = vid_file.relative_to(input_root)
        rel_path_str = str(rel_path)

        # Check if path matches regex (if provided)
        if path_regex is not None and not path_regex.search(rel_path_str):
            continue

        # Add the parent directory to the set of directories to process
        directories_to_process.add(vid_file.parent)

    return directories_to_process


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Batch process INSV files with Gyroflow stabilization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EXAMPLE_USAGE,
    )
    parser.add_argument("input_root",
                        help="Directory containing subdirectories with INSV and GPX files")
    parser.add_argument("output_root", help="Directory where processed results will be saved")
    parser.add_argument(
        "--path-regex",
        help="Regex pattern for input video paths relative to input_root (default: all videos)")
    parser.add_argument("--gyroflow-cmd",
                        default="gyroflow",
                        help="Gyroflow command/path (default: 'gyroflow')")
    parser.add_argument("--keep-backward",
                        action="store_true",
                        help="Keep backward-looking video streams instead of deleting them")
    parser.add_argument("--no-render",
                        action="store_true",
                        help="Process files without rendering video output (dry run)")
    return parser.parse_args()


def main():
    args = parse_arguments()

    input_root = Path(args.input_root)
    output_root = Path(args.output_root)

    # Validate input directory
    if not input_root.is_dir():
        print(f"Error: Input directory '{input_root}' does not exist or is not a directory")
        sys.exit(1)

    # Compile regex pattern if provided
    path_regex = re.compile(args.path_regex) if args.path_regex else None
    # Gyroflow CLI requires absolute paths
    output_root_abs = output_root.resolve()

    # Find directories containing video files that match the path regex
    directories_to_process = find_directories_to_process(input_root, path_regex)

    # Process each directory that has matching videos
    for rec_dir in sorted(directories_to_process):
        process_directory(rec_dir, output_root_abs, args.gyroflow_cmd, DEFAULT_PRESET,
                          args.keep_backward, args.no_render)

    print("Batch processing completed!")


if __name__ == "__main__":
    main()
