import importlib.util
from pathlib import Path


def _probe():
    path = Path(__file__).parents[1] / "tools" / "msfs_weather_probe.py"
    spec = importlib.util.spec_from_file_location("msfs_weather_probe", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_msfs_probe_distinguishes_radar_rendering_from_readback(tmp_path):
    (tmp_path / "MapView.h").write_text(
        "FsTextureId fsMapViewCreate(FsContext ctx, unsigned x, unsigned y);\n"
        "void fsMapViewSetWeatherRadarV2(FsTextureId id);\n"
        "void fsRenderUpdateTexture(FsTextureId id, void *data);\n",
        encoding="utf-8")
    report = _probe().scan(tmp_path)
    assert report["documented_rendering_path"]
    assert not report["documented_numeric_readback"]


def test_msfs_probe_would_recognise_a_real_pixel_readback(tmp_path):
    (tmp_path / "MapView.h").write_text(
        "FsTextureId fsMapViewCreate(FsContext ctx, unsigned x, unsigned y);\n"
        "void fsMapViewSetWeatherRadarV2(FsTextureId id);\n"
        "bool fsRenderReadPixels(FsTextureId id, void *out);\n",
        encoding="utf-8")
    report = _probe().scan(tmp_path)
    assert report["documented_numeric_readback"]
