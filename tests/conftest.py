"""Shared fixtures + requirements traceability plugin.

Tests declare which requirement they verify with @pytest.mark.req("REQ-xx").
After the run, reports/traceability.md maps every requirement to its tests and
their results, and flags requirements that have no test.
"""
import tomllib
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import pytest

from drivesim import Drive, DriveConfig, Simulation

ROOT = Path(__file__).resolve().parent.parent
REQUIREMENTS = tomllib.loads((ROOT / "requirements.toml").read_text(encoding="utf-8"))
REPORT = ROOT / "reports" / "traceability.md"

_results: dict[str, dict[str, str]] = defaultdict(dict)   # req -> {test id: outcome}


# --------------------------------------------------------------------- fixtures
@pytest.fixture
def make_drive():
    """Factory: make_drive(**config_overrides) -> (drive, simulation)."""
    def _make(**overrides):
        drive = Drive(DriveConfig(**overrides))
        return drive, Simulation(drive, dt=1e-4)
    return _make


# ------------------------------------------------------------ traceability hooks
def pytest_configure(config):
    config.addinivalue_line("markers", "req(id, ...): requirement(s) verified by this test")


def pytest_collection_modifyitems(config, items):
    for item in items:
        for marker in item.iter_markers("req"):
            for req in marker.args:
                if req not in REQUIREMENTS:
                    raise pytest.UsageError(f"{item.nodeid} references unknown requirement {req}")


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    reqs = [r for m in item.iter_markers("req") for r in m.args]
    if not reqs:
        return
    if report.when == "call" or (report.when == "setup" and report.outcome != "passed"):
        result = {"passed": "PASS", "failed": "FAIL", "skipped": "SKIP"}[report.outcome]
        for req in reqs:
            _results[req][item.nodeid.split("::", 1)[1]] = result


def pytest_sessionfinish(session, exitstatus):
    if not _results:          # e.g. only unit tests selected
        return
    REPORT.parent.mkdir(exist_ok=True)
    lines = [
        "# Requirements Traceability Matrix",
        "",
        f"Generated {datetime.now():%Y-%m-%d %H:%M} from `requirements.toml` and the pytest run.",
        "",
        "| Requirement | Title | Category | Tests | Result |",
        "|---|---|---|---|---|",
    ]
    covered = passed = 0
    for req_id, req in REQUIREMENTS.items():
        tests = _results.get(req_id, {})
        if not tests:
            verdict, test_list = "⚠️ NOT TESTED", "—"
        else:
            covered += 1
            ok = all(r == "PASS" for r in tests.values())
            passed += ok
            verdict = "✅ PASS" if ok else "❌ FAIL"
            test_list = "<br>".join(f"`{name}` {r}" for name, r in tests.items())
        lines.append(f"| **{req_id}** | {req['title']} | {req['category']} | {test_list} | {verdict} |")
    total = len(REQUIREMENTS)
    lines += ["", f"**Coverage:** {covered}/{total} requirements have tests · "
                  f"**Verified:** {passed}/{total} requirements pass all their tests", ""]
    REPORT.write_text("\n".join(lines), encoding="utf-8")


def pytest_terminal_summary(terminalreporter):
    if _results:
        terminalreporter.write_sep("-", f"traceability matrix written to {REPORT.relative_to(ROOT)}")
