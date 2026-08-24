# SPDX-License-Identifier: CERN-OHL-S-2.0
# Copyright (c) 2026 Jan IngenHousz Institute

import copy
import importlib.util
import json
import shutil
import tempfile
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "release_guard", ROOT / "tools" / "release_guard.py")
release_guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release_guard)


class ReleaseGuardTests(unittest.TestCase):
    def setUp(self):
        self.temporary = Path(tempfile.mkdtemp(prefix="ambit-release-guard-"))
        self.assets = self.temporary / "assets"
        self.assets.mkdir()
        self.version = "1.1.4"
        self.tag = "v1.1.4"
        self.repository = "Jan-IngenHousz-Institute/ambit"
        self.target_sha = "a" * 40
        self.app = "ambit-fw-v1.1.4.bin"
        payloads = {
            "bootloader.bin": b"bootloader",
            "partitions.bin": b"partitions",
            "boot_app0.bin": b"boot-app-zero",
            self.app: b"qualified-application",
        }
        for name, payload in payloads.items():
            (self.assets / name).write_bytes(payload)
        flash = []
        offsets = {
            "bootloader.bin": "0x0",
            "partitions.bin": "0x8000",
            "boot_app0.bin": "0xe000",
            self.app: "0x10000",
        }
        for name in ("bootloader.bin", "partitions.bin", "boot_app0.bin", self.app):
            path = self.assets / name
            flash.append({
                "file": name,
                "offset": offsets[name],
                "size": path.stat().st_size,
                "sha256": release_guard.sha256(path),
            })
        manifest = {
            "name": "ambit-iot",
            "version": self.version,
            "chip": "esp32c3",
            "flash": flash,
            "ota": {"file": self.app},
        }
        (self.assets / "manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8")
        self.candidate = release_guard.verify_candidate(self.assets, self.version)

    def tearDown(self):
        shutil.rmtree(self.temporary)

    def write_json(self, name, value):
        path = self.temporary / name
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def release(self, phase="draft"):
        labels = dict(release_guard.EXPECTED_LABELS)
        labels[self.app] = "Application image (OTA / 0x10000)"
        assets = []
        published = phase == "published"
        release_slug = self.tag if published else "untagged-114-draft"
        for index, name in enumerate(release_guard.expected_assets(self.version), 10):
            expected = self.candidate[name]
            assets.append({
                "id": index,
                "name": name,
                "label": labels[name],
                "state": "uploaded",
                "size": expected["size"],
                "digest": "sha256:{}".format(expected["sha256"]),
                "created_at": "2026-08-07T10:00:00Z",
                "updated_at": "2026-08-07T10:00:01Z",
                "browser_download_url": "https://github.com/{}/releases/download/{}/{}".format(
                    self.repository, release_slug, name),
            })
        html_url = ("https://github.com/{}/releases/tag/{}".format(
                    self.repository, self.tag) if published else
                    "https://github.com/{}/releases/tag/untagged-114-draft".format(
                        self.repository))
        return {
            "id": 114,
            "node_id": "release-node-114",
            "tag_name": self.tag,
            "target_commitish": "main",
            "name": self.tag,
            "draft": not published,
            "prerelease": False,
            "immutable": published,
            "created_at": "2026-08-07T09:59:59Z",
            "published_at": "2026-08-07T10:01:00Z" if published else None,
            "html_url": html_url,
            "url": "https://api.github.com/repos/{}/releases/114".format(self.repository),
            "assets": assets,
        }

    def ref(self):
        return {"ref": "refs/tags/{}".format(self.tag),
                "object": {"type": "commit", "sha": self.target_sha}}

    def test_candidate_requires_exact_files_manifest_and_qualified_app_hash(self):
        app_hash = self.candidate[self.app]["sha256"]
        verified = release_guard.verify_candidate(
            self.assets, self.version, app_hash)
        self.assertEqual(verified[self.app]["sha256"], app_hash)
        (self.assets / "release-preview.md").write_text("not a release asset", encoding="utf-8")
        with self.assertRaises(release_guard.GuardError):
            release_guard.verify_candidate(self.assets, self.version, app_hash)

    def test_manifest_hash_drift_fails(self):
        manifest_path = self.assets / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["flash"][3]["sha256"] = "0" * 64
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaises(release_guard.GuardError):
            release_guard.verify_candidate(self.assets, self.version)

    def test_draft_proof_survives_publish_only_when_assets_are_unchanged(self):
        downloads = self.temporary / "downloads"
        shutil.copytree(self.assets, downloads)
        releases_path = self.write_json("draft-releases.json", [[self.release()]])
        ref_path = self.write_json("ref.json", self.ref())
        proof_path = self.temporary / "proof.json"
        app_hash = self.candidate[self.app]["sha256"]
        release_guard.make_proof(types.SimpleNamespace(
            assets_dir=str(self.assets), downloads_dir=str(downloads),
            releases_json=str(releases_path), ref_json=str(ref_path),
            repository=self.repository, version=self.version, tag=self.tag,
            target_sha=self.target_sha, branch="main", output=str(proof_path),
            pr_number=4, pr_head_sha="b" * 40, run_id=400, artifact_id=401,
            app_sha256=app_hash))

        published = self.release("published")
        published_path = self.write_json("published-releases.json", [[published]])
        latest_path = self.write_json("latest.json", published)
        release_guard.verify_proof(types.SimpleNamespace(
            proof=str(proof_path), releases_json=str(published_path),
            ref_json=str(ref_path), phase="published",
            latest_json=str(latest_path), downloads_dir=str(downloads)))

        drifted = copy.deepcopy(published)
        drifted["assets"][0]["updated_at"] = "2026-08-07T10:02:00Z"
        drifted_path = self.write_json("drifted.json", [[drifted]])
        with self.assertRaises(release_guard.GuardError):
            release_guard.verify_proof(types.SimpleNamespace(
                proof=str(proof_path), releases_json=str(drifted_path),
                ref_json=str(ref_path), phase="published",
                latest_json=str(latest_path), downloads_dir=str(downloads)))

    def test_existing_tag_or_release_fails_preflight(self):
        releases = self.write_json("existing-release.json", [[self.release()]])
        refs = self.write_json("existing-ref.json", [self.ref()])
        with self.assertRaises(release_guard.GuardError):
            release_guard.preflight(types.SimpleNamespace(
                releases_json=str(releases), refs_json=str(refs), tag=self.tag))

    def test_draft_accepts_repo_scoped_untagged_url_and_rejects_other_repo(self):
        draft = self.release("draft")
        release_guard.verify_release_metadata(
            draft, self.repository, self.tag, "main", "draft")
        draft["html_url"] = "https://github.com/other/project/releases/untagged/114-draft"
        with self.assertRaises(release_guard.GuardError):
            release_guard.verify_release_metadata(
                draft, self.repository, self.tag, "main", "draft")

    def test_published_release_requires_canonical_tag_url(self):
        published = self.release("published")
        release_guard.verify_release_metadata(
            published, self.repository, self.tag, "main", "published")
        published["html_url"] = "https://github.com/{}/releases/untagged/114-draft".format(
            self.repository)
        with self.assertRaises(release_guard.GuardError):
            release_guard.verify_release_metadata(
                published, self.repository, self.tag, "main", "published")

    def test_draft_asset_urls_must_match_temporary_release_namespace(self):
        draft = self.release("draft")
        release_guard.release_assets(
            draft, self.repository, self.tag, self.candidate, "draft")
        draft["assets"][0]["browser_download_url"] = (
            "https://github.com/{}/releases/download/untagged-other/{}".format(
                self.repository, draft["assets"][0]["name"]))
        with self.assertRaises(release_guard.GuardError):
            release_guard.release_assets(
                draft, self.repository, self.tag, self.candidate, "draft")

    def test_anonymous_asset_lines_use_canonical_published_urls(self):
        proof = {
            "repository": self.repository,
            "tag": self.tag,
            "assets": {name: {"id": index, "browser_download_url": "draft-url"}
                       for index, name in enumerate(
                           release_guard.expected_assets(self.version), 10)},
        }
        proof_path = self.write_json("asset-lines-proof.json", proof)
        output = []
        original_stdout = release_guard.sys.stdout
        release_guard.sys.stdout = types.SimpleNamespace(write=output.append)
        try:
            release_guard.asset_lines(types.SimpleNamespace(
                proof=str(proof_path), mode="browser"))
        finally:
            release_guard.sys.stdout = original_stdout
        text = "".join(output)
        self.assertNotIn("draft-url", text)
        self.assertIn("/releases/download/{}/{}".format(self.tag, self.app), text)

    def test_artifact_selection_is_by_exact_id_response_not_timestamp(self):
        artifact = {
            "id": 401,
            "name": "firmware-{}".format("b" * 40),
            "expired": False,
            "workflow_run": {"id": 400},
        }
        path = self.write_json("artifact.json", artifact)
        # A single exact-ID API response is accepted. A list with a duplicate
        # candidate is rejected instead of selecting the latest timestamp.
        args = types.SimpleNamespace(artifacts_json=str(path), name=artifact["name"])
        release_guard.select_artifact(args)
        duplicate = self.write_json("artifacts.json", {"artifacts": [artifact, artifact]})
        with self.assertRaises(release_guard.GuardError):
            release_guard.select_artifact(types.SimpleNamespace(
                artifacts_json=str(duplicate), name=artifact["name"]))


class WorkflowBoundaryTests(unittest.TestCase):
    def test_release_workflow_has_one_draft_producer_and_one_protected_finalizer(self):
        workflow = (ROOT / ".github/workflows/release.yml").read_text(encoding="utf-8")
        stage, finalizer = workflow.split("  publish-finalizer:", 1)
        self.assertIn("draftRelease", (ROOT / ".releaserc.json").read_text(encoding="utf-8"))
        self.assertEqual(stage.count("run: npx semantic-release\n"), 1)
        self.assertNotIn("draft=false", stage)
        self.assertIn("environment: firmware-release", finalizer)
        self.assertEqual(finalizer.count("-F draft=false"), 1)
        self.assertNotIn("npx semantic-release", finalizer)
        self.assertNotIn("actions/upload-artifact", finalizer)
        # Every qualification pin must be sourced from a repository variable --
        # never a literal, never an arbitrary expression. That is the boundary
        # being guarded here. Matched by SHAPE rather than by one release's
        # variable names: pinning the next firmware should not require editing
        # this test, and a stale assertion here fails the PR that re-pins rather
        # than the release it was meant to protect.
        for pin in ("PR_HEAD_SHA", "PR_RUN_ID", "ARTIFACT_ID", "APP_SHA256"):
            self.assertRegex(
                workflow,
                r"QUALIFIED_%s: \$\{\{ vars\.AMBIT_V\d+_%s \}\}" % (pin, pin))
        self.assertNotIn("actions/artifacts?name=", workflow)
        self.assertIn("-F draft=false -f make_latest=true", finalizer)
        preview = stage.index("npx semantic-release --dry-run")
        preflight = stage.index("release_guard.py preflight")
        publish = stage.index("run: npx semantic-release\n")
        self.assertLess(preview, preflight)
        self.assertLess(preflight, publish)
        preflight_step = stage[stage.rfind("      - name:", 0, preflight):preflight]
        self.assertIn("if: steps.preview.outputs.will-release == 'true'", preflight_step)
        self.assertIn("attestations: read", finalizer)
        self.assertIn("gh --version", finalizer)

    def test_plugin_and_host_compile_flags_are_pinned(self):
        package = json.loads((ROOT / "package.json").read_text(encoding="utf-8"))
        self.assertEqual(package["devDependencies"]["@semantic-release/github"], "11.0.6")
        config = json.loads((ROOT / ".releaserc.json").read_text(encoding="utf-8"))
        github_plugin = next(item for item in config["plugins"]
                             if isinstance(item, list) and item[0] == "@semantic-release/github")
        self.assertIs(github_plugin[1]["draftRelease"], True)
        self.assertIs(github_plugin[1]["addReleases"], False)
        pr_workflow = (ROOT / ".github/workflows/pr.yml").read_text(encoding="utf-8")
        self.assertGreaterEqual(pr_workflow.count("-std=c++11 -Wall -Wextra -Werror"), 2)
        self.assertIn("semantic_release_github_draft.mjs", pr_workflow)
        self.assertIn("install_release_tooling.sh", pr_workflow)
        proof = (ROOT / "test/release_process/semantic_release_github_draft.mjs").read_text(
            encoding="utf-8")
        self.assertIn('readFile(".releaserc.json", "utf8")', proof)
        installer = (ROOT / "tools/install_release_tooling.sh").read_text(encoding="utf-8")
        self.assertIn("4bd998c6530867151587e99ca8abb7ede0d1e51904195f8945caf2d1eed22554",
                      installer)


if __name__ == "__main__":
    unittest.main()
