# Local publication audit — attributed-history restoration candidate

This record defines the local candidate gate. It authorizes no push, PR, tag, release,
workflow, hardware, private-repository, or downstream change.

## Object scope

- Existing public base: `c81c928c77849833fec08a50fd2dc1d5d7a5749f`.
- Unsanitized private source (comparison only; excluded from transfer): `8126a5f2ae174848d40d5b12f464da1f24bbb055`.
- Sanitized parallel replay tip: `1cb4e60c2f33579ded7fb29b715a5dd97ee73f7f`.
- Candidate ref: `refs/heads/history/restoration-candidate`; its exact object ID is resolved and recorded in the separately retained private audit package after the merge exists.
- Intended transfer refspec: only `refs/heads/history/restoration-candidate:refs/heads/history/restoration-candidate`; no tag or wildcard refspec.
- `git rev-list --objects history/restoration-candidate --not refs/remotes/public/main` yields a conservative review and push superset of 96 objects / 33 blobs, including the candidate object. It is not the literal wire pack or exact server-missing set: four historical wrapper blobs are already reachable from pinned public `main`, while 92 objects / 29 blobs are not reachable from that main tip; negotiation may omit further objects available through other server refs.
- A commit cannot self-contain its own ID. The candidate ID, raw superset manifest, pack, bundle, SHA-256 attestations, 47-name inventory, and 4/29 reachability proof are retained in a separate external audit package supplied to independent reviewers. That package is not published at a dead in-repository path; public readers can inspect this record and `MIGRATION_MAP.md` directly.

## Sanitization accuracy and buildability

The direct `git commit-tree` replay never prunes empty or degenerate commits. It removes only the ADPD6000 vendor prefix and clears notebook output/execution fields; all other historical blobs are copied by object ID. Commit signatures are omitted because their object IDs changed.

Rewritten commits 1–22 are intentionally non-buildable because retained application files still include the removed `lib/ADPD6000/adi_adpd6000.h`. Commit 23 introduces the clean-room replacement, and commits 23–24 no longer have the dangling vendor include. The complete retained set is 47 interface/register/API names: every name occurs in four historical wrapper blobs already reachable from pinned public `main`; four names—`ADPD6000`, `adi_adpd6000`, `API_ADPD6000_ERROR_OK`, and `ADPD6000_ERROR_RETURN`—recur as quotations in the two new audit Markdown files; zero occur in any genuinely new non-documentation blob. No excluded vendor implementation body is introduced.

The publication owner has technically dispositioned the already-public wrapper interface/register/API names as preservable for truthful attributed history while excluding vendor implementation bodies. This is a technical scope decision, not legal advice or a license grant.

The preserved merge subjects `#1`, `#2`, and `#3` identify private `ambit-iot` PRs. Any GitHub auto-link to the same number in public `ambit` is unrelated.

## Machine-enforced local gates

The private audit runner exits nonzero unless all of these checks pass:

1. Both remote pins match; a fresh reconstruction byte-matches the approved 24-row mapping and replay tip without creating or moving a replay ref.
2. Author/committer identities and raw timestamps, message bytes, ordering, parent counts, mapped topology, and all three merges match.
3. Every replay tree lacks the excluded vendor prefix; notebook source arrays match while outputs, execution counts, widgets, attachments, and cell metadata are absent.
4. Commits 1–22 have the disclosed missing vendor include and commits 23–24 do not; the independently derived 47-name set is exact, all 47 occur in four already-public wrappers, only four names recur in new documentation, and zero occur in new non-documentation blobs.
5. The candidate has the pinned public first parent, approved replay second parent, accountable JII author/committer, and a tree differing from public only by this file and `MIGRATION_MAP.md`.
6. Existing public tag objects/types/peels match remote state; historical private tag targets map to rewritten commits without creating local tag refs.
7. The conservative 96-object review/push superset and pack contain no original private commit, excluded ADPD blob, or original notebook blob; their four already-public / 92 not-reachable split is asserted, and strict fsck, pack verification, and portable bundle verification pass.
8. Pinned Gitleaks 8.28.0 reports zero findings both for Git patches and for all 33 reviewed blobs—29 not reachable from pinned public `main` plus four already-public wrappers; the count and byte total are recorded.
9. The private audit package lives outside the worktree; both `git status --porcelain` and `git add -A --dry-run` are empty.

## Human-reviewed inventories

The runner produces contributor identities, privacy/device and URL candidates, the complete residual identifier/interface inventory, and MIME/size inventories. Contributor-identity privacy approval was supplied by the user's explicit choice to preserve exact attribution and execute publication. The publication-owner technical disposition permits the 47 already-public wrapper names to remain for truthful history while excluding vendor implementation bodies. That disposition is not legal advice or a license grant.

## Reproducible toolchain

- `git version 2.55.0`
- `jq-1.8.1`
- `Python 3.14.6`
- `file-5.46`
- Gitleaks `8.28.0`; release archive SHA-256 `a65b5253807a68ac0cafa4414031fd740aeb55f54fb7e55f386acb52e6a840eb`; extracted binary SHA-256 `5fd1b3b0073269484d40078662e921d07427340ab9e6ed526ccd215a565b3298`.

The separately supplied private audit package contains the approved mapping, deterministic reconstruction generator, documentation generator, single verification entry point, exact commands, raw inventories, checksums, pack, and portable bundle. Each run uses a new output directory and does not delete prior evidence.

## Review boundary and residuals

Contributor-identity privacy approval and the 47-name publication-owner technical disposition are resolved for this corrective lane. Original signatures cannot survive rewritten object IDs and remain transparently omitted. Ticket 01 remains open for broader work but is not a dependency of this sanitized ticket-12 lane. A fresh independent review of the exact rebuilt candidate and later hosted-graph review remain mandatory. No public operation is authorized by this local audit.
