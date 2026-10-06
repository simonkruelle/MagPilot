"""Package the current local CAD files without raw experiments or backups."""
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent


def main():
    required = ['README.md','parameters.json','build_pen.py','verify_stl.py',
                'validation.json','stl_validation.json','magnetic_pen_v1.blend',
                'preview.png','drawing.svg','drawing.pdf','drawing.png']
    for name in required:
        if not (HERE/name).is_file():
            raise ValueError('Missing deliverable: '+name)
    files = [p for p in HERE.rglob('*') if p.is_file() and
             p.suffix in ('.md','.json','.py','.blend','.png','.svg','.pdf','.stl') and
             '__pycache__' not in p.parts]
    bundle = HERE/'magnetic_pen_v1.zip'
    with zipfile.ZipFile(bundle,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(files):
            archive.write(path,arcname='magnetic_pen_v1/'+str(path.relative_to(HERE)))
    with zipfile.ZipFile(bundle) as archive:
        if archive.testzip() is not None:
            raise ValueError('Bundle failed CRC check.')
    print('Packaged',len(files),'files:',bundle.name,'(',bundle.stat().st_size,'bytes )')


if __name__ == '__main__':
    main()
