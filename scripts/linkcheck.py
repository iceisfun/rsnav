#!/usr/bin/env python3
"""Resolve every relative markdown link in docs/ against the filesystem, including #anchors."""
import os, re, sys

ROOT = "/data/work/rsnav"
DOCS = os.path.join(ROOT, "docs")
LINK = re.compile(r'\[([^\]]*)\]\(([^)]+)\)')
HEAD = re.compile(r'^(#{1,6})\s+(.*?)\s*$')


def slug(text):
    """GitHub's heading -> anchor transform."""
    text = re.sub(r'<[^>]+>', '', text)
    # inline code / emphasis markers contribute nothing
    text = text.replace('`', '')
    text = text.lower()
    text = re.sub(r'[^\w\s-]', '', text)   # drop punctuation, keep word chars/space/hyphen
    text = text.strip().replace(' ', '-')
    return text


def anchors_of(path):
    seen, out = {}, set()
    with open(path) as fh:
        fence = False
        for line in fh:
            if line.lstrip().startswith('```'):
                fence = not fence
                continue
            if fence:
                continue
            m = HEAD.match(line)
            if not m:
                continue
            a = slug(m.group(2))
            n = seen.get(a, 0)
            seen[a] = n + 1
            out.add(a if n == 0 else f"{a}-{n}")
    return out


anchor_cache = {}
bad = []
files = anchors = 0

for name in sorted(os.listdir(DOCS)):
    if not name.endswith(".md"):
        continue
    path = os.path.join(DOCS, name)
    for lineno, line in enumerate(open(path), 1):
        for _t, target in LINK.findall(line):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            filepart, _, frag = target.partition("#")
            resolved = os.path.normpath(os.path.join(DOCS, filepart)) if filepart else path
            if filepart:
                files += 1
                if not os.path.exists(resolved):
                    bad.append((name, lineno, target, "no such file"))
                    continue
            if frag and resolved.endswith(".md"):
                anchors += 1
                if resolved not in anchor_cache:
                    anchor_cache[resolved] = anchors_of(resolved)
                if frag not in anchor_cache[resolved]:
                    bad.append((name, lineno, target, "no such anchor"))

print(f"checked {files} file links and {anchors} anchors")
for b in bad:
    print(f"BROKEN {b[0]}:{b[1]}  {b[2]}  ({b[3]})")
print(f"{len(bad)} broken")
sys.exit(1 if bad else 0)
