# AGENTS.md

This file defines long-term agent boundaries for this repository.

- Keep source, examples and documentation portable. Do not commit machine-specific absolute paths, private datasets, build outputs, debug outputs, logs, generated documentation, credentials, API keys, or personal notes.
- Prefer small, verifiable changes. When changing C++ code, run the CMake configure/build path when the required dependencies are available.
- Treat files under `data/`, `debug/`, `results/`, `logs/`, `build/`, `install/`, and generated `docs/html/` as local artifacts unless explicitly requested otherwise.
- Use `README.md` for stable user-facing setup and usage information. Put architecture notes under `docs/`.
- Keep examples focused on reproducible public datasets or placeholder paths that users can replace locally.
