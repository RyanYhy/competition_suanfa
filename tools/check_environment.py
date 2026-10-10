#!/usr/bin/env python3
"""Offline installation checks; --inference also runs the actual CPU models."""
import argparse
import hashlib
import importlib
import json
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL_NAMES = ('doll_best.onnx', 'plate_best.onnx', 'traffic_light_best.onnx')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inference', action='store_true')
    parser.add_argument('--quiet', action='store_true', help='Only print failures and the final status')
    args = parser.parse_args()
    failures = []

    def check(name, action):
        try:
            detail = action()
            if not args.quiet:
                print('[OK] {}{}'.format(name, ': ' + str(detail) if detail is not None else ''))
        except Exception as exc:
            failures.append(name)
            print('[FAIL] {}: {}'.format(name, exc))

    def require(condition, detail):
        if not condition:
            raise RuntimeError(detail)

    check('Python', lambda: require(sys.version_info[:2] == (3, 8), 'Use ROS Noetic Python 3.8; got ' + sys.version))
    if not args.quiet:
        print('Platform: {} / Python {}'.format(platform.platform(), sys.executable))
    for module in ('rospy', 'rospkg', 'cv_bridge', 'cv2', 'numpy', 'PIL', 'yaml',
                   'onnxruntime', 'rapidocr_onnxruntime', 'pyclipper', 'shapely',
                   'coloredlogs', 'flatbuffers', 'packaging', 'google.protobuf', 'sympy', 'six'):
        check(module, lambda name=module: importlib.import_module(name).__file__)

    def numpy_version():
        import numpy
        require(numpy.__version__ == '1.24.4', 'Run tools/setup.sh; NumPy must be 1.24.4 for this release')
        return numpy.__version__

    check('NumPy compatibility', numpy_version)

    def package_paths():
        import rospkg
        pack = rospkg.RosPack()
        for name in ('smart_community_sim', 'jie_ware', 'my_local_planner'):
            resolved = Path(pack.get_path(name)).resolve()
            require(resolved == ROOT / name, 'Another overlay shadows {}: {}'.format(name, resolved))
        return str(ROOT)

    check('ROS package paths', package_paths)
    check('Chinese font', lambda: require(Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').is_file(), 'Install fonts-noto-cjk'))
    for name in MODEL_NAMES:
        check(name, lambda name=name: require((ROOT/'smart_community_sim/model'/name).stat().st_size > 1024 * 1024, 'Missing model or Git LFS pointer; obtain the full ONNX file'))

    def runtime_assets():
        import yaml
        package = ROOT / 'smart_community_sim'
        for name in ('field_reference', 'competition_keepout'):
            path = package / 'maps' / (name + '.yaml')
            image = yaml.safe_load(path.read_text())['image']
            require((path.parent / image).is_file(), 'Missing map: ' + image)
        for item in json.loads((package/'config/recognition_assets.json').read_text()):
            actual = hashlib.sha256((package/item['texture']).read_bytes()).hexdigest()
            require(actual == item['sha256'], 'Texture changed: ' + item['texture'])
        return 'reference/keepout maps and 21 original recognition textures'

    check('Assets', runtime_assets)
    if args.inference:
        sys.path.insert(0, str(ROOT/'smart_community_sim/src'))

        def infer():
            import cv2
            import numpy as np
            from smart_community_sim.yolo_ocr import PlateOcr, YoloOnnx
            package = ROOT / 'smart_community_sim'
            for name in MODEL_NAMES:
                model = YoloOnnx(package/'model'/name)
                model.detect(np.zeros((480, 640, 3), dtype=np.uint8))
                print('  {}: input {}x{}, classes {}'.format(name, model.width, model.height, model.names))
            image = cv2.imread(str(package/'materials/textures/plate_3.png'))
            result = PlateOcr().recognize(image)
            require(any(item['text'] for item in result), 'OCR produced no text on the bundled example plate')
            return result

        check('YOLO + OCR CPU inference', infer)
    if failures:
        print('Failed checks: ' + ', '.join(failures))
        print('Activate the workspace .venv and source devel/setup.bash, or rerun tools/setup.sh.')
        return 1
    print('Installation checks passed. Live sensor/navigation verification requires Gazebo.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
