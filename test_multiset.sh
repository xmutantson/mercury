#!/bin/sh
# Canonical release-test failure multiset normalizer.
# Usage: test_multiset.sh LOGFILE
#        command-that-runs-tests | test_multiset.sh -
# Only the text after the final '] FAIL:' marker is part of the multiset.

set -eu

if [ "$#" -ne 1 ]; then
	printf 'usage: %s LOGFILE|-\n' "$0" >&2
	exit 2
fi

input=$1
tmp=${TMPDIR:-/tmp}/mercury-test-multiset.$$
trap 'rm -f "$tmp"' EXIT HUP INT TERM

if [ "$input" = "-" ]; then
	sed -n 's/^.*\] FAIL:[[:space:]]*//p' >"$tmp"
else
	sed -n 's/^.*\] FAIL:[[:space:]]*//p' "$input" >"$tmp"
fi

LC_ALL=C sort "$tmp" -o "$tmp"
count=$(wc -l <"$tmp" | tr -d '[:space:]')
hash=$(sha256sum "$tmp" | awk '{print $1}')

printf 'count=%s\nsha256=%s\n' "$count" "$hash"
cat "$tmp"
