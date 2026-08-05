# Origin and continuation history

This repository is a genuine GitHub fork of
[`hjc2023/ambit`](https://github.com/hjc2023/ambit). The upstream v1 work and its authorship
remain visible in the inherited Git history and are not re-authored or presented as JII's
later work.

The first JII continuation snapshot is based on upstream commit
`64b3bf72a757b594e61fd48398dd1db9a893fa93` and imports the maintained firmware tree from
JII private commit `bebd67b6e4141588a7a8943be70c4ee3ebd3ee41`. The private
repository's commits are deliberately not grafted onto the public fork: the snapshot records
a new continuation boundary without inventing historical ancestry.

The continuation removes the ADPD6000 vendor SDK subtree from the current source tree and
uses a JII-owned driver written against public hardware-interface facts. This removal does
not rewrite or conceal objects already present in the inherited upstream history. Modified
inherited files retain their existing author notices; JII-authored files carry JII copyright
and SPDX identifiers. Other bundled third-party components retain their own notices.
