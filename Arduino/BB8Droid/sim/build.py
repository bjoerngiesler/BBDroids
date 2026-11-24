#!/usr/bin/env python3
"""Builds the simulation page from page.html (template) and sim.js (core).

  python3 build.py            -> BB8-DriveSimulation-online.html (three.js from the CDN)
  python3 build.py --offline  -> BB8-DriveSimulation.html (three.js embedded;
                                 three.min.js r128 must be next to this script,
                                 e.g. from `npm pack three@0.128.0` -> package/build/three.min.js)
"""
import pathlib
import sys

here = pathlib.Path(__file__).parent
page = (here / "page.html").read_text(encoding="utf-8")
sim = (here / "sim.js").read_text(encoding="utf-8")
html = page.replace("/*SIMJS*/", sim)
cdn = '<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>'

if "--offline" in sys.argv:
    three = (here / "three.min.js").read_text(encoding="utf-8")
    html = html.replace(cdn, "<script>/* three.js r128, MIT License, (c) 2010-2021 three.js authors */\n" + three + "\n</script>")
    out = here / "BB8-DriveSimulation.html"
else:
    out = here / "BB8-DriveSimulation-online.html"

out.write_text(html, encoding="utf-8")
print("written:", out)
