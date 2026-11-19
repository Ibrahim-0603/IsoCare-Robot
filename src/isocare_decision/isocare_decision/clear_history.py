"""Forget every recorded dose, so rehearsals start clean.

Run:  ros2 run isocare_decision clear_dose_history [--db PATH]
"""

import argparse
import os

from isocare_decision.dose_history import DoseHistory

DEFAULT_DB = os.path.expanduser('~/.isocare/dose_history.db')


def main(argv=None):
    """Clear the dose history file and say how many doses were removed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', default=DEFAULT_DB, help='history file (default %(default)s)')
    args = parser.parse_args(argv)
    removed = DoseHistory(args.db).clear()
    print(f'removed {removed} recorded dose(s) from {args.db}')


if __name__ == '__main__':
    main()
