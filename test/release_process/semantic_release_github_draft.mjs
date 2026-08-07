// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

import assert from "node:assert/strict";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";

const packageJson = JSON.parse(
  await (await import("node:fs/promises")).readFile(
    "node_modules/@semantic-release/github/package.json", "utf8"));
assert.equal(packageJson.version, "11.0.6", "the tested plugin must be exactly 11.0.6");

const packageRoot = path.resolve("node_modules/@semantic-release/github");
const publishGitHub = (await import(pathToFileURL(
  path.join(packageRoot, "lib/publish.js")))).default;
const successGitHub = (await import(pathToFileURL(
  path.join(packageRoot, "lib/success.js")))).default;

const calls = [];
class MockOctokit {
  constructor() {}

  async request(route, options) {
    calls.push({ route, options });
    if (typeof route === "object") {
      return { data: { browser_download_url: "https://example.invalid/asset.bin" } };
    }
    if (route === "POST /repos/{owner}/{repo}/releases") {
      return { data: {
        id: 114,
        upload_url: "https://uploads.github.invalid/{?name,label}",
        html_url: "https://github.invalid/release/114",
      } };
    }
    if (route === "GET /repos/{owner}/{repo}") {
      return { data: { full_name: "Jan-IngenHousz-Institute/ambit" } };
    }
    throw new Error(`unexpected Octokit call: ${route}`);
  }
}

const temporary = await mkdtemp(path.join(os.tmpdir(), "ambit-draft-proof-"));
try {
  await writeFile(path.join(temporary, "asset.bin"), Buffer.from("qualified bytes"));
  const logger = { log() {}, warn() {}, error() {} };
  const context = {
    cwd: temporary,
    env: { GITHUB_TOKEN: "mock-token" },
    options: { repositoryUrl: "https://github.com/Jan-IngenHousz-Institute/ambit.git" },
    branch: { name: "main" },
    nextRelease: {
      gitTag: "v1.1.4",
      name: "v1.1.4",
      notes: "qualified release",
      version: "1.1.4",
    },
    logger,
  };

  await publishGitHub(
    { draftRelease: true, assets: [{ path: "asset.bin", label: "qualified" }] },
    context,
    { Octokit: MockOctokit });

  const create = calls.find(call => call.route === "POST /repos/{owner}/{repo}/releases");
  assert.ok(create, "publish must create one GitHub release");
  assert.equal(create.options.draft, true, "publish must create a draft");
  assert.equal(create.options.target_commitish, "main");
  assert.equal(calls.filter(call => typeof call.route === "object").length, 1,
    "publish must upload the configured asset");
  assert.equal(calls.some(call => String(call.route).startsWith("PATCH ")), false,
    "draft publish must return without publishing the release");

  calls.length = 0;
  await successGitHub(
    {
      successComment: false,
      failComment: false,
      failTitle: false,
      releasedLabels: false,
      addReleases: false,
    },
    { ...context, commits: [], releases: [] },
    { Octokit: MockOctokit });
  assert.deepEqual(calls.map(call => call.route), ["GET /repos/{owner}/{repo}"],
    "success phase must not PATCH or publish the draft");
} finally {
  await rm(temporary, { recursive: true, force: true });
}

console.log("@semantic-release/github 11.0.6 draft-only behavior proved");
