#!/usr/bin/env bash
cat bundle/unzipped/Tests/Asm/Test$1.asm | awk '{ if (NF >= 2) { print $1 " " substr($2, 1,1) } else {print $1}}' | sort -u
