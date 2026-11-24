#!/bin/sh
# Runs check.js (or another script using sim.js) without Node, using the JavaScriptCore shell that ships with macOS.
# Usage: ./check-jsc.sh [script.js]   (default check.js; with Node installed, just use: node check.js)
cd "$(dirname "$0")"
SCRIPT=${1:-check.js}
JSC=/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc
SHIM=$(mktemp -t bb8check).js
cat > "$SHIM" <<JS
var module = { exports: {} };
load('sim.js');
var __sim = module.exports;
var require = function () { return __sim; };
var console = { log: function () { print(Array.prototype.join.call(arguments, ' ')); } };
load('$SCRIPT');
JS
"$JSC" "$SHIM"; rc=$?
rm -f "$SHIM"
exit $rc
