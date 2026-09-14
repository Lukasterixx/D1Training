"""Markdown to HTML for the results dashboard, standard library only.

Covers the subset the experiment record and docs use: ATX headings, paragraphs,
nested and task lists, GFM tables, fenced code, blockquotes, rules, HTML
comments (dropped), and inline code, emphasis, strikethrough, inline and
reference links, images and autolinks. Source text is always escaped, so a note
cannot inject markup into the page.
"""
from __future__ import annotations

import html
import re
from typing import Callable

FENCE = re.compile(r"^(\s{0,3})(`{3,}|~{3,})\s*([\w+-]*)")
HEADING = re.compile(r"^\s{0,3}(#{1,6})\s+(.*?)(?:\s+#+)?\s*$")
RULE = re.compile(r"^\s{0,3}([-*_])(?:\s*\1){2,}\s*$")
LIST_ITEM = re.compile(r"^(\s*)([-*+]|\d{1,9}[.)])\s+(.*)$")
QUOTE = re.compile(r"^\s{0,3}>\s?(.*)$")
TABLE_RULE = re.compile(r"^\s*\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)*\|?\s*$")
TASK = re.compile(r"^\[([ xX])\]\s+(.*)$")
REFERENCE = re.compile(r"^\s{0,3}\[([^\]]+)\]:\s*(\S+)(?:\s+\"[^\"]*\")?\s*$")
COMMENT = re.compile(r"<!--.*?-->", re.S)

INLINE = re.compile(
    r"(?P<esc>\\[\\`*_{}\[\]()#+\-.!|~<>])"
    r"|(?P<code>(?P<ticks>`+)(?P<codetext>.+?)(?P=ticks))"
    r"|(?P<image>!\[(?P<alt>[^\]]*)\]\((?P<src>[^)\s]+)(?:\s+\"[^\"]*\")?\))"
    r"|(?P<link>\[(?P<text>(?:[^\[\]]|\[[^\]]*\])+)\]"
    r"(?:\((?P<href>[^)\s]+)(?:\s+\"[^\"]*\")?\)|\[(?P<ref>[^\]]*)\]))"
    r"|(?P<auto><(?P<url>https?://[^>\s]+)>)"
)
EMPHASIS = [
    (re.compile(r"\*\*(?=\S)(.+?)(?<=\S)\*\*", re.S), "strong"),
    (re.compile(r"(?<!\w)__(?=\S)(.+?)(?<=\S)__(?!\w)", re.S), "strong"),
    (re.compile(r"\*(?=\S)(.+?)(?<=\S)\*"), "em"),
    (re.compile(r"(?<!\w)_(?=\S)(.+?)(?<=\S)_(?!\w)"), "em"),
    (re.compile(r"~~(?=\S)(.+?)(?<=\S)~~", re.S), "del"),
]
SAFE_SCHEMES = ("http:", "https:", "mailto:")

Resolver = Callable[[str, str], str]


def _indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def slugify(text: str) -> str:
    text = re.sub(r"[`*_\[\]()]", "", text).lower()
    return re.sub(r"[^a-z0-9]+", "-", text).strip("-") or "section"


def split_row(line: str) -> list[str]:
    """Split a GFM table row on pipes outside code spans; `\\|` is a literal pipe."""
    line = line.strip()
    if line.startswith("|"):
        line = line[1:]
    if line.endswith("|") and not line.endswith("\\|"):
        line = line[:-1]
    cells, current, in_code, i = [], [], False, 0
    while i < len(line):
        char = line[i]
        if char == "\\" and line[i + 1:i + 2] == "|":
            current.append("|")
            i += 2
            continue
        if char == "`":
            in_code = not in_code
        if char == "|" and not in_code:
            cells.append("".join(current).strip())
            current = []
        else:
            current.append(char)
        i += 1
    cells.append("".join(current).strip())
    return cells


class Renderer:
    def __init__(self, resolve: Resolver | None = None):
        self.resolve = resolve or (lambda url, kind: url)
        self.references: dict[str, str] = {}
        self.slugs: dict[str, int] = {}

    # Inline -----------------------------------------------------------------
    def _href(self, url: str, kind: str) -> str:
        if re.match(r"^[a-zA-Z][\w+.-]*:", url) and not url.lower().startswith(SAFE_SCHEMES):
            return "#"
        return self.resolve(url, kind)

    def inline(self, text: str) -> str:
        stash: list[str] = []

        def keep(fragment: str) -> str:
            stash.append(fragment)
            return f"\x00{len(stash) - 1}\x00"

        def token(match: re.Match) -> str:
            if match.group("esc"):
                return keep(html.escape(match.group("esc")[1]))
            if match.group("code"):
                return keep(f"<code>{html.escape(match.group('codetext').strip())}</code>")
            if match.group("image"):
                src = html.escape(self._href(match.group("src"), "image"))
                alt = html.escape(match.group("alt"))
                return keep(f'<img src="{src}" alt="{alt}" loading="lazy">')
            if match.group("link"):
                url = match.group("href")
                if url is None:
                    key = (match.group("ref") or match.group("text")).lower()
                    url = self.references.get(key)
                    if url is None:
                        return keep(html.escape(match.group(0)))
                href = self._href(url, "link")
                external = ' target="_blank" rel="noopener"' if href.startswith(("http:", "https:")) else ""
                return keep(f'<a href="{html.escape(href)}"{external}>{self.inline(match.group("text"))}</a>')
            url = html.escape(match.group("url"))
            return keep(f'<a href="{url}" target="_blank" rel="noopener">{url}</a>')

        text = INLINE.sub(token, text.replace("\x00", ""))
        text = html.escape(text, quote=False)
        for pattern, tag in EMPHASIS:
            text = pattern.sub(rf"<{tag}>\1</{tag}>", text)
        # Stashed fragments can contain placeholders of their own (a link's text).
        while "\x00" in text:
            text = re.sub(r"\x00(\d+)\x00", lambda m: stash[int(m.group(1))], text)
        return text

    # Blocks -----------------------------------------------------------------
    def _table_start(self, lines: list[str], i: int) -> bool:
        return (i + 1 < len(lines) and "|" in lines[i] and "|" in lines[i + 1]
                and bool(TABLE_RULE.match(lines[i + 1])))

    def _interrupts(self, lines: list[str], i: int) -> bool:
        line = lines[i]
        return bool(FENCE.match(line) or HEADING.match(line) or RULE.match(line)
                    or QUOTE.match(line) or LIST_ITEM.match(line) or self._table_start(lines, i))

    def _table(self, lines: list[str], i: int) -> tuple[str, int]:
        header = split_row(lines[i])
        aligns = []
        for cell in split_row(lines[i + 1]):
            left, right = cell.startswith(":"), cell.endswith(":")
            aligns.append("center" if left and right else "right" if right else "left" if left else "")
        i += 2
        rows = []
        while i < len(lines) and lines[i].strip() and "|" in lines[i]:
            rows.append(split_row(lines[i]))
            i += 1

        def cells(values: list[str], tag: str) -> str:
            out = []
            for index in range(len(header)):
                value = values[index] if index < len(values) else ""
                align = aligns[index] if index < len(aligns) else ""
                style = f' style="text-align:{align}"' if align else ""
                out.append(f"<{tag}{style}>{self.inline(value)}</{tag}>")
            return "".join(out)

        body = "".join(f"<tr>{cells(row, 'td')}</tr>" for row in rows)
        table = f"<table><thead><tr>{cells(header, 'th')}</tr></thead><tbody>{body}</tbody></table>"
        return f'<div class="table-wrap">{table}</div>', i

    def _list(self, lines: list[str], i: int) -> tuple[str, int]:
        first = LIST_ITEM.match(lines[i])
        indent, ordered = len(first.group(1)), first.group(2)[0].isdigit()
        items: list[list[str]] = []
        n = len(lines)
        while i < n:
            match = LIST_ITEM.match(lines[i])
            if not match or len(match.group(1)) != indent or match.group(2)[0].isdigit() != ordered:
                break
            width = match.start(3)
            body = [match.group(3)]
            i += 1
            while i < n:
                line = lines[i]
                if not line.strip():
                    j = i
                    while j < n and not lines[j].strip():
                        j += 1
                    if j < n and _indent(lines[j]) > indent:
                        body.extend([""] * (j - i))
                        i = j
                        continue
                    sibling = j < n and LIST_ITEM.match(lines[j])
                    if sibling and len(sibling.group(1)) == indent:
                        body.append("")  # A blank line between items makes the list loose.
                        i = j
                    break
                lead = _indent(line)
                if lead > indent:
                    body.append(line[min(lead, width):])
                    i += 1
                elif LIST_ITEM.match(line) or self._interrupts(lines, i):
                    break
                else:
                    body.append(line.strip())  # Lazy paragraph continuation.
                    i += 1
            items.append(body)

        parts = []
        for body in items:
            task = TASK.match(body[0])
            if task:
                body = [task.group(2)] + body[1:]
            blocks = self.blocks(body)
            if "" not in body:
                blocks = [b[3:-4] if b.startswith("<p>") and b.endswith("</p>") else b for b in blocks]
            content = "\n".join(blocks)
            if task:
                done = task.group(1).lower() == "x"
                box = f'<span class="task-box{" done" if done else ""}" aria-hidden="true"></span>'
                label = "done" if done else "to do"
                parts.append(f'<li class="task{" done" if done else ""}" aria-label="{label}">{box}'
                             f'<div class="task-body">{content}</div></li>')
            else:
                parts.append(f"<li>{content}</li>")
        if ordered:
            start = int(re.match(r"\d+", first.group(2)).group())
            opening = f'<ol start="{start}">' if start != 1 else "<ol>"
            return opening + "".join(parts) + "</ol>", i
        return "<ul>" + "".join(parts) + "</ul>", i

    def blocks(self, lines: list[str]) -> list[str]:
        out, i, n = [], 0, len(lines)
        while i < n:
            line = lines[i]
            if not line.strip():
                i += 1
                continue
            fence = FENCE.match(line)
            if fence:
                marker, lang = fence.group(2), fence.group(3)
                strip, body = len(fence.group(1)), []
                i += 1
                while i < n:
                    closing = lines[i].strip()
                    if closing.startswith(marker) and set(closing) == {marker[0]}:
                        i += 1
                        break
                    body.append(lines[i][min(strip, _indent(lines[i])):])
                    i += 1
                cls = f' class="language-{html.escape(lang)}"' if lang else ""
                out.append(f"<pre><code{cls}>{html.escape(chr(10).join(body))}</code></pre>")
                continue
            heading = HEADING.match(line)
            if heading:
                level, text = len(heading.group(1)), heading.group(2)
                slug = slugify(text)
                count = self.slugs.get(slug, 0)
                self.slugs[slug] = count + 1
                slug = f"{slug}-{count}" if count else slug
                out.append(f'<h{level} id="{slug}">{self.inline(text)}</h{level}>')
                i += 1
                continue
            if RULE.match(line):
                out.append("<hr>")
                i += 1
                continue
            if self._table_start(lines, i):
                table, i = self._table(lines, i)
                out.append(table)
                continue
            if QUOTE.match(line):
                inner = []
                while i < n and QUOTE.match(lines[i]):
                    inner.append(QUOTE.match(lines[i]).group(1))
                    i += 1
                out.append(f"<blockquote>{''.join(self.blocks(inner))}</blockquote>")
                continue
            if LIST_ITEM.match(line):
                block, i = self._list(lines, i)
                out.append(block)
                continue
            paragraph = [line.strip()]
            i += 1
            while i < n and lines[i].strip() and not self._interrupts(lines, i):
                paragraph.append(lines[i].strip())
                i += 1
            out.append(f"<p>{self.inline(chr(10).join(paragraph))}</p>")
        return out


def strip_header(text: str, fields: tuple[str, ...] = ()) -> str:
    """Drop the first `# Title` line and any top-level `**Field:** value` lines,
    for pages whose header already shows them."""
    text = re.sub(r"^\s{0,3}#\s+.*\n?", "", text, count=1, flags=re.M)
    for name in fields:
        text = re.sub(rf"^\*\*{name}:\*\*.*\n?", "", text, count=1, flags=re.M)
    return text


def render(text: str, resolve: Resolver | None = None) -> str:
    """Render Markdown to HTML. `resolve(url, kind)` rewrites link/image targets."""
    renderer = Renderer(resolve)
    lines = []
    for line in COMMENT.sub("", text.replace("\r\n", "\n")).expandtabs(4).split("\n"):
        reference = REFERENCE.match(line)
        if reference:
            renderer.references[reference.group(1).lower()] = reference.group(2)
        else:
            lines.append(line)
    return "\n".join(renderer.blocks(lines))


def title_of(text: str, fallback: str) -> str:
    match = re.search(r"^\s{0,3}#\s+(.+?)\s*$", text, re.M)
    return re.sub(r"[`*_]", "", match.group(1)) if match else fallback
