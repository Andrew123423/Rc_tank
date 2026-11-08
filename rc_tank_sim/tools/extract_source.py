#!/usr/bin/env python3
"""Extract only STL/INO members from the user archive into a flat source folder."""
import argparse,io,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('archive',type=Path);p.add_argument('--out',type=Path,default=Path('source'));a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
count=0
with zipfile.ZipFile(a.archive) as outer:
    for info in outer.infolist():
        if not info.filename.lower().endswith('.zip'):continue
        with zipfile.ZipFile(io.BytesIO(outer.read(info))) as z:
            for member in z.infolist():
                name=Path(member.filename).name
                if Path(name).suffix.lower() not in ('.stl','.ino'):continue
                if member.file_size>100_000_000:raise ValueError('Unexpected member size')
                (a.out/name).write_bytes(z.read(member));count+=1
print(f'Extracted {count} STL/INO files into {a.out}')
