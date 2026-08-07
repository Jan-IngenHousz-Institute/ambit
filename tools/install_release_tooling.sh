#!/usr/bin/env bash
# SPDX-License-Identifier: CERN-OHL-S-2.0
# Copyright (c) 2026 Jan IngenHousz Institute

set -euo pipefail

# npm validates registry integrity itself. This additional archive pin makes the
# exact plugin source qualified by semantic_release_github_draft.mjs explicit
# and reviewable even though this firmware repository does not commit a lockfile.
PLUGIN_ARCHIVE_SHA256="4bd998c6530867151587e99ca8abb7ede0d1e51904195f8945caf2d1eed22554"
RELEASE_TOOLING_TEMP=$(mktemp -d)
trap 'rm -rf "${RELEASE_TOOLING_TEMP}"' EXIT

pushd "${RELEASE_TOOLING_TEMP}" >/dev/null
PLUGIN_ARCHIVE=$(npm pack @semantic-release/github@11.0.6 --silent)
test "${PLUGIN_ARCHIVE}" = "semantic-release-github-11.0.6.tgz"
echo "${PLUGIN_ARCHIVE_SHA256}  ${PLUGIN_ARCHIVE}" | sha256sum --check --status
popd >/dev/null

npm install --ignore-scripts --package-lock=false
node -e "const p=require('./node_modules/@semantic-release/github/package.json'); if(p.version!=='11.0.6') process.exit(1)"
