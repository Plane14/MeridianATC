from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.config import Config


@pytest.fixture
def cli():
    pytest.importorskip("rich")
    from meridianatc import cli

    return cli


def test_setup_downloads_the_speech_model_and_device_the_engine_will_use(cli, monkeypatch, tmp_path):
    from meridianatc.audio import stt, voices
    from meridianatc.navdata import build, fetch

    settings = Config()
    settings.speech.model = "small.en"
    settings.speech.languages = ["fr"]
    settings.speech.compute_type = "float32"
    settings.speech.device = "cpu"
    monkeypatch.setattr(cli.Config, "load", lambda *args: settings)
    monkeypatch.setattr(fetch, "fetch_all", Mock())
    monkeypatch.setattr(voices, "download_voices", Mock())
    monkeypatch.setattr(build, "DB_PATH", tmp_path)
    recognizer = Mock()
    monkeypatch.setattr(stt, "Recognizer", recognizer)
    assert cli.cmd_setup(SimpleNamespace(force=False, vatglasses=False, all_voices=False,
                                         hq=False, config=None)) == 0
    recognizer.assert_called_once_with(model_size=stt.engine_model(settings), languages=["fr"],
                                       device="cpu", compute_type="float32")
    recognizer.return_value.warm.assert_called_once()


def test_frequency_import_uses_custom_dll_and_refuses_xplane(cli, monkeypatch):
    from meridianatc.integrations import facilities
    from meridianatc.navdata import localdata

    settings = Config()
    settings.sim.simconnect_dll = "custom-SimConnect.dll"
    lookup = Mock(return_value=None)
    monkeypatch.setattr(facilities, "find_simconnect", lookup)
    assert cli._frequencies_from_sim("KJFK", settings) == 1
    lookup.assert_called_once_with("custom-SimConnect.dll")
    lookup.return_value = "custom-SimConnect.dll"
    client = Mock()
    constructor = Mock(return_value=client)
    monkeypatch.setattr(facilities, "SimFacilities", constructor)
    monkeypatch.setattr(localdata, "replace_airport", Mock(return_value=(2, 1)))
    assert cli._frequencies_from_sim("KJFK", settings) == 0
    constructor.assert_called_once_with(dll_path="custom-SimConnect.dll")
    client.close.assert_called_once()
    settings.sim.simulator = "xplane"
    lookup.reset_mock()
    assert cli._frequencies_from_sim("KJFK", settings) == 1
    lookup.assert_not_called()


@pytest.mark.parametrize("command", ["console", "gui"])
@pytest.mark.parametrize("failure", ["warm", "start"])
def test_startup_failure_still_closes_the_engine(cli, monkeypatch, command, failure):
    pytest.importorskip("scipy")
    if command == "gui":
        pytest.importorskip("fastapi")
    from meridianatc import engine as engines

    instance = Mock()
    getattr(instance, failure).side_effect = RuntimeError("startup failed")
    monkeypatch.setattr(engines, "Engine", Mock(return_value=instance))
    monkeypatch.setattr(cli, "NavDB", Mock())
    monkeypatch.setattr(cli.Config, "load", lambda *args: Config())
    monkeypatch.setattr(cli.logs, "configure", Mock())
    args = cli.build_parser().parse_args([command, "--no-sim"])
    with pytest.raises(RuntimeError, match="startup failed"):
        args.func(args)
    instance.stop.assert_called_once()


@pytest.mark.parametrize("port", ["-1", "0", "65536", "not-a-number"])
def test_gui_rejects_invalid_ports_before_starting_any_resources(cli, port):
    with pytest.raises(SystemExit) as error:
        cli.build_parser().parse_args(["gui", "--port", port])
    assert error.value.code == 2


def test_yaml_parse_diagnostics_do_not_print_the_bad_line_or_its_secret(cli, tmp_path, capsys):
    path = tmp_path / "config.yaml"
    path.write_text('api_key: "synthetic-private-value\n', encoding="utf-8")
    assert cli.main(["--config", str(path), "doctor"]) == 1
    output = capsys.readouterr()
    assert "Invalid YAML" in output.out and "line" in output.out
    assert "synthetic-private-value" not in output.out + output.err
