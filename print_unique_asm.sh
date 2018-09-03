#!/usr/bin/env bash
cat tests/asm/Test$1.asm | awk '{ if ($0 !~ /:/ ){ if (NF >= 2) { print $1 " " substr($2, 1,1) } else {print $1}}}' | sort -u
