"""Last-dose history of the decision_engine: a small SQLite file.

One row per (patient, drug) holds the epoch time of the last dose that was really given. The
file survives a restart, which matters because a 6 h interval must survive a reboot.
"""

import os
import sqlite3
import threading


class DoseHistory:
    """Last dose time per patient and drug."""

    def __init__(self, path):
        """Open (and create) the database. ':memory:' is allowed for tests."""
        if path != ':memory:':
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._lock = threading.Lock()
        self._db = sqlite3.connect(path, check_same_thread=False)
        with self._lock, self._db:
            self._db.execute('CREATE TABLE IF NOT EXISTS doses ('
                             'patient_id TEXT NOT NULL, drug TEXT NOT NULL, '
                             'given_at REAL NOT NULL, PRIMARY KEY (patient_id, drug))')

    def last(self, patient_id, drug):
        """Epoch seconds of the last dose, or None if there is none."""
        with self._lock:
            row = self._db.execute('SELECT given_at FROM doses WHERE patient_id=? AND drug=?',
                                   (patient_id, drug)).fetchone()
        return None if row is None else float(row[0])

    def record(self, patient_id, drug, when):
        """Remember that a dose was given at `when` (replaces an older one)."""
        with self._lock, self._db:
            self._db.execute('INSERT OR REPLACE INTO doses VALUES (?, ?, ?)',
                             (patient_id, drug, float(when)))

    def clear(self):
        """Forget every dose (rehearsals). Returns how many rows were removed."""
        with self._lock, self._db:
            return self._db.execute('DELETE FROM doses').rowcount
