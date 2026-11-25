#!/usr/bin/env python3
"""Builds the simulation pages from their templates and the shared cores.

  page.html         + sim.js + course.js           -> BB8-DriveSimulation[-online].html
  compare-page.html + sim.js + cary.js + course.js -> BB8-DriveComparison[-online].html  (BBDroids vs Cary Christie V1)

  python3 build.py            -> *-online.html (three.js from the CDN)
  python3 build.py --offline  -> three.js embedded; three.min.js r128 must be next to this script,
                                 e.g. from `npm pack three@0.128.0` -> package/build/three.min.js

Always rebuild both after changing sim.js, so the comparison page runs the same BB-8 core.
"""
import pathlib
import sys

here = pathlib.Path(__file__).parent
cores = {
    "/*SIMJS*/": (here / "sim.js").read_text(encoding="utf-8"),
    "/*CARYJS*/": (here / "cary.js").read_text(encoding="utf-8"),
    "/*COURSEJS*/": (here / "course.js").read_text(encoding="utf-8"),
}
cdn = '<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>'
offline = "--offline" in sys.argv
if offline:
    three = (here / "three.min.js").read_text(encoding="utf-8")
    three_tag = "<script>/* three.js r128, MIT License, (c) 2010-2021 three.js authors */\n" + three + "\n</script>"

for template, name in [("page.html", "BB8-DriveSimulation"), ("compare-page.html", "BB8-DriveComparison")]:
    html = (here / template).read_text(encoding="utf-8")
    for placeholder, code in cores.items():
        html = html.replace(placeholder, code)
    if offline:
        assert cdn in html, template
        html = html.replace(cdn, three_tag)
    out = here / (name + (".html" if offline else "-online.html"))
    out.write_text(html, encoding="utf-8")
    print("written:", out)
