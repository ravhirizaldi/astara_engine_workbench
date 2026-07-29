"""Launch the ASTARA Engineering Workbench."""

from aerospace_workbench.ui import show_workbench


def main() -> None:
    if not show_workbench():
        print("ASTARA GUI unavailable. Use `awb --help` for CLI commands.")


if __name__ == "__main__":
    main()
