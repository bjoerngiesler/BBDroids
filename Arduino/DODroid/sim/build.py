#!/usr/bin/env python3
"""Builds the D-O simulation page from its template and sources.

  page.html + do-model.js + dosim.js + ../../BB8Droid/sim/course.js -> DO-Drive-Simulation[-online].html

  python3 build.py            -> DO-Drive-Simulation-online.html (three.js from the CDN)
  python3 build.py --offline  -> DO-Drive-Simulation.html (three.js embedded; three.min.js r128 must be next to
                                 this script, e.g. from `npm pack three@0.128.0` -> package/build/three.min.js,
                                 or extracted from the existing offline HTML)
"""
import pathlib
import sys

here = pathlib.Path(__file__).parent
html = (here / "page.html").read_text(encoding="utf-8")
html = html.replace("/*DOMODEL*/", (here / "do-model.js").read_text(encoding="utf-8"))
html = html.replace("/*DOSIMJS*/", (here / "dosim.js").read_text(encoding="utf-8"))
# obstacle course: shared with the BB-8 simulation (one source)
html = html.replace("/*COURSEJS*/", (here / ".." / ".." / "BB8Droid" / "sim" / "course.js").read_text(encoding="utf-8"))
cdn = '<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>'

if "--offline" in sys.argv:
    three = (here / "three.min.js").read_text(encoding="utf-8")
    assert cdn in html
    html = html.replace(cdn, "<script>/* three.js r128, MIT License, (c) 2010-2021 three.js authors */\n" + three + "\n</script>")
    out = here / "DO-Drive-Simulation.html"
else:
    out = here / "DO-Drive-Simulation-online.html"

out.write_text(html, encoding="utf-8")
print("written:", out)
