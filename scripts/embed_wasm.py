import base64
import json
import pathlib
import sys

WASM_PLACEHOLDER = "__WASM_BASE64__"
RESULTS_PLACEHOLDER = "__RESULTS_JSON__"

def main() -> int:
    if len(sys.argv) != 5:
        print("usage: embed_wasm.py TEMPLATE.html SIM.wasm RESULTS.json OUTPUT.html", file=sys.stderr)
        return 2
    template = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    wasm = pathlib.Path(sys.argv[2]).read_bytes()
    results = json.loads(pathlib.Path(sys.argv[3]).read_text(encoding="utf-8"))
    for placeholder in (WASM_PLACEHOLDER, RESULTS_PLACEHOLDER):
        if template.count(placeholder) != 1:
            print(f"expected exactly one {placeholder} in {sys.argv[1]}", file=sys.stderr)
            return 1
    encoded = base64.b64encode(wasm).decode("ascii")
    html = template.replace(WASM_PLACEHOLDER, encoded)
    html = html.replace(RESULTS_PLACEHOLDER, json.dumps(results, separators=(",", ":")))
    pathlib.Path(sys.argv[4]).write_text(html, encoding="utf-8")
    print(f"wrote {sys.argv[4]} ({len(wasm)} bytes of WebAssembly, {len(results['maps'])} maps of results)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
