from pathlib import Path

Import("env")

TARGET = "            self.metavar = f\"[{self.type.get_metavar(None) or self.type.name.upper()}]\""
REPLACEMENT = '''            try:
                metavar = self.type.get_metavar(None, None)
            except TypeError:
                metavar = self.type.get_metavar(None)
            self.metavar = f"[{metavar or self.type.name.upper()}]"'''  # noqa: E501


def patch_esptool() -> None:
    tool_dir = env.PioPlatform().get_package_dir("tool-esptoolpy")
    if not tool_dir:
        return

    cli_util = Path(tool_dir) / "esptool" / "cli_util.py"
    if not cli_util.exists():
        return

    contents = cli_util.read_text()
    if "get_metavar(None, None)" in contents:
        return

    if TARGET not in contents:
        return

    cli_util.write_text(contents.replace(TARGET, REPLACEMENT))


patch_esptool()
