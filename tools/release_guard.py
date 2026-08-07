#!/usr/bin/env python3
"""Fail-closed validation for the AMBIT immutable release boundary.

The workflow deliberately keeps GitHub API access in shell steps and all
policy/byte validation here.  This makes the important checks deterministic,
unit-testable, and reusable on both sides of the protected environment gate.
"""

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from urllib.parse import urlparse


EXPECTED_OFFSETS = {
    "bootloader.bin": "0x0",
    "partitions.bin": "0x8000",
    "boot_app0.bin": "0xe000",
}
EXPECTED_LABELS = {
    "manifest.json": "Flash manifest (offsets + sha256)",
    "bootloader.bin": "Bootloader (0x0)",
    "partitions.bin": "Partition table (0x8000)",
    "boot_app0.bin": "OTA data init (0xe000)",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class GuardError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise GuardError(message)


def load(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def dump(value, path=None):
    text = json.dumps(value, indent=2, sort_keys=True) + "\n"
    if path:
        Path(path).write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)


def flatten_pages(value):
    if isinstance(value, dict):
        return value["artifacts"] if "artifacts" in value else [value]
    require(isinstance(value, list), "expected a JSON list")
    if value and all(isinstance(page, list) for page in value):
        return [item for page in value for item in page]
    return value


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_assets(version):
    app = "ambit-fw-v{}.bin".format(version)
    return ["manifest.json", app, "bootloader.bin", "partitions.bin", "boot_app0.bin"]


def verify_candidate(directory, version, expected_app_sha256=None):
    root = Path(directory)
    app = "ambit-fw-v{}.bin".format(version)
    names = expected_assets(version)
    require(root.is_dir(), "candidate asset directory is missing")
    actual = sorted(path.name for path in root.iterdir() if path.is_file())
    require(actual == sorted(names),
            "candidate must contain exactly five assets: expected {}; got {}".format(
                sorted(names), actual))

    manifest = load(root / "manifest.json")
    require(set(manifest) == {"name", "version", "chip", "flash", "ota"},
            "manifest top-level keys do not match the release contract")
    require(manifest["name"] == "ambit-iot", "manifest name must be ambit-iot")
    require(manifest["version"] == version, "manifest version mismatch")
    require(manifest["chip"] == "esp32c3", "manifest chip mismatch")
    require(manifest["ota"] == {"file": app}, "manifest OTA target mismatch")

    expected_flash = ["bootloader.bin", "partitions.bin", "boot_app0.bin", app]
    require(isinstance(manifest["flash"], list), "manifest flash field must be a list")
    require([entry.get("file") for entry in manifest["flash"]] == expected_flash,
            "manifest flash order/file allowlist mismatch")

    result = {}
    for name in names:
        path = root / name
        result[name] = {"size": path.stat().st_size, "sha256": sha256(path)}
    if expected_app_sha256 is not None:
        require(SHA256_RE.match(expected_app_sha256) is not None,
                "qualified application sha256 is malformed")
        require(result[app]["sha256"] == expected_app_sha256,
                "application image does not match the qualified sha256")

    for entry in manifest["flash"]:
        require(set(entry) == {"file", "offset", "size", "sha256"},
                "manifest flash entry keys do not match the release contract")
        name = entry["file"]
        expected_offset = "0x10000" if name == app else EXPECTED_OFFSETS[name]
        require(entry["offset"] == expected_offset,
                "manifest offset mismatch for {}".format(name))
        require(entry["size"] == result[name]["size"],
                "manifest size mismatch for {}".format(name))
        require(isinstance(entry["sha256"], str) and SHA256_RE.match(entry["sha256"]),
                "manifest sha256 is malformed for {}".format(name))
        require(entry["sha256"] == result[name]["sha256"],
                "manifest sha256 mismatch for {}".format(name))
    return result


def exact_release(releases, tag):
    matches = [release for release in flatten_pages(releases)
               if release.get("tag_name") == tag]
    require(len(matches) == 1,
            "expected exactly one release for {}; found {}".format(tag, len(matches)))
    return matches[0]


def exact_ref(ref_json, tag, target_sha):
    require(ref_json.get("ref") == "refs/tags/{}".format(tag), "tag ref name mismatch")
    target = ref_json.get("object", {})
    require(target.get("type") == "commit", "release tag must be a lightweight commit tag")
    require(target.get("sha") == target_sha, "release tag points at the wrong commit")


def verify_release_metadata(release, repository, tag, branch, phase):
    require(isinstance(release.get("id"), int) and release["id"] > 0,
            "release id is missing")
    require(release.get("tag_name") == tag, "release tag metadata mismatch")
    require(release.get("target_commitish") == branch,
            "release target_commitish must be {}".format(branch))
    require(release.get("name") == tag, "release name must equal the tag")
    require(release.get("prerelease") is False, "release must be stable, not a prerelease")
    expected_api_url = "https://api.github.com/repos/{}/releases/{}".format(
        repository, release["id"])
    require(release.get("url") == expected_api_url, "release API identity mismatch")
    html_url = release.get("html_url", "")
    if phase == "draft":
        require(release.get("draft") is True, "staged release must remain a draft")
        require(release.get("immutable") is False, "draft release must not claim immutability")
        require(release.get("published_at") is None, "draft must not have a publication timestamp")
        parsed = urlparse(html_url)
        expected_prefix = "/{}/releases/".format(repository)
        require(parsed.scheme == "https" and parsed.netloc == "github.com"
                and parsed.path.startswith(expected_prefix)
                and len(parsed.path) > len(expected_prefix)
                and not parsed.params and not parsed.query and not parsed.fragment,
                "draft HTML URL is not a repository-scoped HTTPS release URL")
    elif phase == "published":
        require(release.get("draft") is False, "final release must not be a draft")
        require(release.get("immutable") is True, "published release is not immutable")
        require(isinstance(release.get("published_at"), str) and release["published_at"],
                "published release is missing published_at")
        require(html_url == "https://github.com/{}/releases/tag/{}".format(
                    repository, tag), "published release HTML URL is not canonical")
    else:
        raise GuardError("unknown release phase")


def release_assets(release, repository, tag, candidate):
    assets = release.get("assets")
    require(isinstance(assets, list), "release assets are missing")
    expected_names = set(candidate)
    actual_names = [asset.get("name") for asset in assets]
    require(len(actual_names) == len(set(actual_names)), "release contains duplicate asset names")
    require(set(actual_names) == expected_names and len(actual_names) == 5,
            "release must contain the exact five-asset allowlist")

    by_name = {}
    ids = set()
    app = next(name for name in expected_names if name.startswith("ambit-fw-v"))
    labels = dict(EXPECTED_LABELS)
    labels[app] = "Application image (OTA / 0x10000)"
    for asset in assets:
        name = asset["name"]
        require(asset.get("state") == "uploaded", "asset {} is not uploaded".format(name))
        require(isinstance(asset.get("id"), int) and asset["id"] > 0,
                "asset {} has no REST id".format(name))
        require(asset["id"] not in ids, "release contains duplicate asset ids")
        ids.add(asset["id"])
        require(asset.get("size") == candidate[name]["size"],
                "REST size mismatch for {}".format(name))
        require(asset.get("digest") == "sha256:{}".format(candidate[name]["sha256"]),
                "REST digest mismatch for {}".format(name))
        require(asset.get("label") == labels[name], "asset label mismatch for {}".format(name))
        require(isinstance(asset.get("created_at"), str) and asset["created_at"],
                "asset {} is missing created_at".format(name))
        require(isinstance(asset.get("updated_at"), str) and asset["updated_at"],
                "asset {} is missing updated_at".format(name))
        expected_url = "https://github.com/{}/releases/download/{}/{}".format(
            repository, tag, name)
        require(asset.get("browser_download_url") == expected_url,
                "asset browser URL mismatch for {}".format(name))
        by_name[name] = {
            "id": asset["id"],
            "name": name,
            "size": asset["size"],
            "digest": asset["digest"],
            "sha256": candidate[name]["sha256"],
            "created_at": asset["created_at"],
            "updated_at": asset["updated_at"],
            "browser_download_url": asset["browser_download_url"],
        }
    return by_name


def verify_downloads(directory, assets):
    root = Path(directory)
    actual = sorted(path.name for path in root.iterdir() if path.is_file()) if root.is_dir() else []
    require(actual == sorted(assets), "download directory does not contain the exact asset set")
    for name, expected in assets.items():
        path = root / name
        require(path.stat().st_size == expected["size"],
                "downloaded size mismatch for {}".format(name))
        require(sha256(path) == expected["sha256"],
                "downloaded sha256 mismatch for {}".format(name))


def make_proof(args):
    candidate = verify_candidate(args.assets_dir, args.version, args.app_sha256)
    releases = load(args.releases_json)
    release = exact_release(releases, args.tag)
    ref_json = load(args.ref_json)
    exact_ref(ref_json, args.tag, args.target_sha)
    verify_release_metadata(release, args.repository, args.tag, args.branch, "draft")
    assets = release_assets(release, args.repository, args.tag, candidate)
    verify_downloads(args.downloads_dir, assets)
    proof = {
        "schema": 1,
        "repository": args.repository,
        "version": args.version,
        "tag": args.tag,
        "target_sha": args.target_sha,
        "target_commitish": args.branch,
        "qualified_pr": {
            "number": args.pr_number,
            "head_sha": args.pr_head_sha,
            "run_id": args.run_id,
            "artifact_id": args.artifact_id,
            "app_sha256": args.app_sha256,
        },
        "release": {
            key: release.get(key) for key in
            ("id", "node_id", "tag_name", "target_commitish", "name",
             "prerelease", "created_at", "html_url", "url")
        },
        "assets": assets,
    }
    dump(proof, args.output)


def verify_proof(args):
    proof = load(args.proof)
    require(proof.get("schema") == 1, "unsupported proof schema")
    require(proof.get("tag") == "v{}".format(proof.get("version")),
            "proof version/tag mismatch")
    qualified = proof.get("qualified_pr", {})
    require(isinstance(qualified.get("number"), int) and qualified["number"] > 0,
            "proof qualified PR number is missing")
    require(isinstance(qualified.get("run_id"), int) and qualified["run_id"] > 0,
            "proof qualified run id is missing")
    require(isinstance(qualified.get("artifact_id"), int) and qualified["artifact_id"] > 0,
            "proof qualified artifact id is missing")
    require(re.match(r"^[0-9a-f]{40}$", qualified.get("head_sha", "")) is not None,
            "proof qualified PR head is malformed")
    app = "ambit-fw-v{}.bin".format(proof["version"])
    require(qualified.get("app_sha256") == proof.get("assets", {}).get(app, {}).get("sha256"),
            "proof application hash is not tied to the qualified hash")
    releases = load(args.releases_json)
    release = exact_release(releases, proof["tag"])
    exact_ref(load(args.ref_json), proof["tag"], proof["target_sha"])
    verify_release_metadata(release, proof["repository"], proof["tag"],
                            proof["target_commitish"], args.phase)
    for key, value in proof["release"].items():
        # GitHub may expose a draft under an untagged URL and replace it with
        # the canonical tag URL when the same release is published.
        if args.phase == "published" and key == "html_url":
            continue
        require(release.get(key) == value, "release identity changed at {}".format(key))

    candidate = {name: {"size": asset["size"], "sha256": asset["sha256"]}
                 for name, asset in proof["assets"].items()}
    current_assets = release_assets(release, proof["repository"], proof["tag"], candidate)
    require(current_assets == proof["assets"], "release asset REST proof changed")
    if args.downloads_dir:
        verify_downloads(args.downloads_dir, proof["assets"])
    if args.phase == "published":
        require(args.latest_json, "published verification requires latest release metadata")
        latest = load(args.latest_json)
        require(latest.get("id") == release["id"] and latest.get("tag_name") == proof["tag"],
                "published release is not the stable latest release")


def preflight(args):
    releases = flatten_pages(load(args.releases_json))
    refs = flatten_pages(load(args.refs_json))
    release_count = sum(1 for release in releases if release.get("tag_name") == args.tag)
    ref_count = sum(1 for ref in refs if ref.get("ref") == "refs/tags/{}".format(args.tag))
    require(release_count == 0, "release {} already exists".format(args.tag))
    require(ref_count == 0, "tag {} already exists".format(args.tag))


def verify_qualified_pr(args):
    pr = load(args.pr_json)
    require(pr.get("number") == args.number, "qualified PR number mismatch")
    require(pr.get("state") == "closed" and pr.get("merged_at"),
            "qualified PR is not merged")
    require(pr.get("base", {}).get("ref") == args.branch,
            "qualified PR base branch mismatch")
    require(pr.get("head", {}).get("sha") == args.head_sha,
            "qualified PR head changed after qualification")


def select_artifact(args):
    artifacts = flatten_pages(load(args.artifacts_json))
    matches = [artifact for artifact in artifacts
               if artifact.get("name") == args.name and artifact.get("expired") is False]
    require(len(matches) == 1,
            "expected one live PR artifact named {}; found {}".format(args.name, len(matches)))
    artifact = matches[0]
    run_id = artifact.get("workflow_run", {}).get("id")
    require(isinstance(run_id, int) and run_id > 0, "artifact workflow run id is missing")
    dump({"artifact_id": artifact["id"], "run_id": run_id})


def verify_run(args):
    run = load(args.run_json)
    require(run.get("id") == args.run_id, "workflow run id mismatch")
    require(run.get("event") == "pull_request", "artifact was not produced by pull_request CI")
    require(run.get("status") == "completed" and run.get("conclusion") == "success",
            "artifact workflow run was not successful")
    require(run.get("head_sha") == args.head_sha, "artifact workflow ran at the wrong PR head")
    require(run.get("path") == ".github/workflows/pr.yml", "artifact came from the wrong workflow")


def asset_lines(args):
    proof = load(args.proof)
    for name in sorted(proof["assets"]):
        asset = proof["assets"][name]
        locator = str(asset["id"]) if args.mode == "api" else asset["browser_download_url"]
        sys.stdout.write("{}\t{}\n".format(locator, name))


def release_asset_lines(args):
    release = exact_release(load(args.releases_json), args.tag)
    assets = release.get("assets", [])
    expected = set(expected_assets(args.version))
    require(len(assets) == 5 and {asset.get("name") for asset in assets} == expected,
            "draft does not have the exact five-asset allowlist")
    for asset in sorted(assets, key=lambda item: item["name"]):
        require(isinstance(asset.get("id"), int) and asset["id"] > 0,
                "release asset id is missing")
        sys.stdout.write("{}\t{}\n".format(asset["id"], asset["name"]))


def parser():
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest="command", required=True)

    candidate = commands.add_parser("candidate")
    candidate.add_argument("--assets-dir", required=True)
    candidate.add_argument("--version", required=True)
    candidate.add_argument("--expected-app-sha256")

    before = commands.add_parser("preflight")
    before.add_argument("--releases-json", required=True)
    before.add_argument("--refs-json", required=True)
    before.add_argument("--tag", required=True)

    qualified_pr = commands.add_parser("verify-qualified-pr")
    qualified_pr.add_argument("--pr-json", required=True)
    qualified_pr.add_argument("--number", required=True, type=int)
    qualified_pr.add_argument("--head-sha", required=True)
    qualified_pr.add_argument("--branch", required=True)

    artifact = commands.add_parser("select-artifact")
    artifact.add_argument("--artifacts-json", required=True)
    artifact.add_argument("--name", required=True)

    run = commands.add_parser("verify-run")
    run.add_argument("--run-json", required=True)
    run.add_argument("--run-id", required=True, type=int)
    run.add_argument("--head-sha", required=True)

    proof = commands.add_parser("make-proof")
    proof.add_argument("--assets-dir", required=True)
    proof.add_argument("--downloads-dir", required=True)
    proof.add_argument("--releases-json", required=True)
    proof.add_argument("--ref-json", required=True)
    proof.add_argument("--repository", required=True)
    proof.add_argument("--version", required=True)
    proof.add_argument("--tag", required=True)
    proof.add_argument("--target-sha", required=True)
    proof.add_argument("--branch", required=True)
    proof.add_argument("--pr-number", required=True, type=int)
    proof.add_argument("--pr-head-sha", required=True)
    proof.add_argument("--run-id", required=True, type=int)
    proof.add_argument("--artifact-id", required=True, type=int)
    proof.add_argument("--app-sha256", required=True)
    proof.add_argument("--output", required=True)

    verify = commands.add_parser("verify-proof")
    verify.add_argument("--proof", required=True)
    verify.add_argument("--releases-json", required=True)
    verify.add_argument("--ref-json", required=True)
    verify.add_argument("--phase", choices=("draft", "published"), required=True)
    verify.add_argument("--latest-json")
    verify.add_argument("--downloads-dir")

    lines = commands.add_parser("asset-lines")
    lines.add_argument("--proof", required=True)
    lines.add_argument("--mode", choices=("api", "browser"), required=True)

    release_lines = commands.add_parser("release-asset-lines")
    release_lines.add_argument("--releases-json", required=True)
    release_lines.add_argument("--tag", required=True)
    release_lines.add_argument("--version", required=True)
    return root


def main():
    args = parser().parse_args()
    if args.command == "candidate":
        dump(verify_candidate(args.assets_dir, args.version, args.expected_app_sha256))
    elif args.command == "preflight":
        preflight(args)
    elif args.command == "verify-qualified-pr":
        verify_qualified_pr(args)
    elif args.command == "select-artifact":
        select_artifact(args)
    elif args.command == "verify-run":
        verify_run(args)
    elif args.command == "make-proof":
        make_proof(args)
    elif args.command == "verify-proof":
        verify_proof(args)
    elif args.command == "asset-lines":
        asset_lines(args)
    elif args.command == "release-asset-lines":
        release_asset_lines(args)


if __name__ == "__main__":
    try:
        main()
    except (GuardError, KeyError, OSError, ValueError, TypeError) as error:
        sys.stderr.write("release guard: {}\n".format(error))
        sys.exit(1)
