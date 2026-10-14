#!/usr/bin/env python3
"""Embed the compiled simulator (WebAssembly) into the HTML page as base64.

Usage: embed_wasm.py TEMPLATE.html SIM.wasm OUTPUT.html

The result is one self-contained file that opens straight from disk, with no server.
"""
import base64
import pathlib
import sys

PLACEHOLDER = "__WASM_BASE64__"


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    template = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    wasm = pathlib.Path(sys.argv[2]).read_bytes()
    if template.count(PLACEHOLDER) != 1:
        print(f"expected exactly one {PLACEHOLDER} in {sys.argv[1]}", file=sys.stderr)
        return 1
    encoded = base64.b64encode(wasm).decode("ascii")
    pathlib.Path(sys.argv[3]).write_text(template.replace(PLACEHOLDER, encoded), encoding="utf-8")
    print(f"wrote {sys.argv[3]} ({len(wasm)} bytes of WebAssembly)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
