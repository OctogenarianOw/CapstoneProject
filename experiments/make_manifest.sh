#!/bin/bash
# Write CAMPAIGN_DIR/MANIFEST.tsv: SHA-256, size in bytes and path of every
# file of the campaign, then check it. Annexe A.1 asks each campaign for a
# manifest so its raw data can be verified on any copy of the repository.
# __pycache__ is skipped: it is a local cache, ignored by git.
#   bash make_manifest.sh CAMPAIGN_DIR
set -e
cd "$1"
{
  printf 'sha256\tbytes\tpath\n'
  find . -type f ! -name MANIFEST.tsv ! -path '*/__pycache__/*' -printf '%P\n' \
    | LC_ALL=C sort | while IFS= read -r p; do
        printf '%s\t%s\t%s\n' "$(sha256sum "$p" | cut -d' ' -f1)" \
          "$(stat -c %s "$p")" "$p"
      done
} > MANIFEST.tsv
tail -n +2 MANIFEST.tsv | awk -F'\t' '{print $1 "  " $3}' | sha256sum -c --quiet
echo "$(basename "$PWD"): $(($(wc -l < MANIFEST.tsv) - 1)) files, manifest verified"
