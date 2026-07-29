# Packaging decision

ASTARA is supported as a repository-run engineering workbench, not as a
distributable wheel.

The Python bridge builds and loads the C++ flight core from `flight_core/`, and
the default configuration resolves repository-owned scenario and vehicle
documents. A wheel containing only the Python package would therefore be
incomplete.

Use an editable installation from a checkout:

```bash
python -m pip install -r requirements.lock
python -m pip install -e . --no-deps
```

A distributable wheel should be introduced only with an explicit native-library
bundling policy, platform-specific wheel builds, packaged configuration
resources, and installed-resource lookup that no longer depends on repository
paths.
