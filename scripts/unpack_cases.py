"""Restore the complete saved synthetic map/query records."""
import gzip
import json
from pathlib import Path

path=Path("reports/cases.json.gz")
payload=gzip.decompress(path.read_bytes())
records=json.loads(payload)
if len(records)!=200:raise ValueError("Expected the published 200-case record set")
target=path.with_suffix("")
if target.exists() and target.read_bytes()!=payload:
    raise ValueError("Refusing to overwrite a different local case record")
target.write_bytes(payload)
print(f"Restored {len(records)} cases to {target}")
