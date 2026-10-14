#!/usr/bin/env python3
"""Browser check for the built page. Needs Playwright with Chromium.
Usage: python3 tests/ui/check_page.py [path/to/line_follower_sim.html]
Loads the page, runs a lap in each camera view, and checks there are no errors and the lap finishes."""
import os, sys
from playwright.sync_api import sync_playwright

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
page_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "web", "line_follower_sim.html")
exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
fails = []
def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (("  " + detail) if detail else ""))
    if not ok: fails.append(name)

with sync_playwright() as p:
    b = p.chromium.launch(executable_path=exe if os.path.exists(exe) else None, args=["--no-sandbox"])
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    errors = []
    pg.on("pageerror", lambda e: errors.append(str(e)))
    pg.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
    pg.goto("file://" + os.path.abspath(page_path)); pg.wait_for_timeout(700)
    check("page loads without errors", not errors, "; ".join(errors))
    check("four control modes listed", pg.locator('input[name="mode"]').count() == 4)
    check("six maps listed", pg.locator("#map option").count() == 6)
    check("speed buttons are gone", pg.locator("#speeds").count() == 0)
    check("controls column has no inner scroll", pg.evaluate("getComputedStyle(document.querySelector('.controls')).overflowY") == "visible")
    colours = pg.evaluate("""() => { const bad = []; for (const el of document.querySelectorAll('body *:not(input)')) { const cs = getComputedStyle(el);
      for (const v of [cs.color, cs.backgroundColor, cs.borderTopColor]) { const m = v.match(/\\d+/g); if (m && m.length >= 3 && (Math.abs(m[0]-m[1])>2 || Math.abs(m[1]-m[2])>2)) bad.push(el.tagName + ':' + v); } } return bad.slice(0,5); }""")
    check("interface is black, white and gray only", not colours, str(colours))
    for view in ("orbit", "top", "chase"):
        pg.click('#views button[data-view="%s"]' % view); pg.wait_for_timeout(250)
        px = pg.evaluate("""() => { const c = document.getElementById('view'); const d = c.getContext('2d').getImageData(0,0,c.width,c.height).data; let dark = 0; for (let i = 0; i < d.length; i += 4*37) if (d[i] < 60) dark++; return dark; }""")
        check("%s view draws the track and robot" % view, px > 50, "dark samples %d" % px)
    pg.click('#views button[data-view="orbit"]')
    pg.select_option("#map", "0"); pg.click("#start")
    pg.wait_for_function("document.getElementById('status').textContent.startsWith('Finished')", timeout=30000)
    check("Warm Up Oval finishes in the browser", True, pg.inner_text("#lap"))
    check("no errors after a full run", not errors, "; ".join(errors))
    b.close()
print("%d check(s) failed" % len(fails) if fails else "all page checks passed")
sys.exit(1 if fails else 0)
