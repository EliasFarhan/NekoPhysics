#!/usr/bin/env python3
"""Checks the links of the MrDocs HTML reference built by the `docs` target.

Usage: python3 docs/check_links.py build/debug/docs [--json report.json]

Errors (exit code 1):
  - dead link: the target file or #anchor does not exist;
  - mismatch: the link text names another symbol than the target page;
  - constructor link: a link in prose lands on a constructor or destructor
    page (`@ref Self` written inside the comment of class Self does that);
  - internal: a public page names `internal::`;
  - code span: a code span ends with punctuation (`@p name.` does that, use
    backticks when punctuation follows the name).

To review (exit code 0): names of documented types (classes, structs, enums,
aliases) shown as plain text. In prose it is either ordinary English ("Index
of the root node") or a missing `@ref`. In a synopsis it is a MrDocs
limitation (initializers, template arguments of std types): name the type
with `@ref` in the comment instead. `@code` blocks are never linked and are
not reported.
"""

import argparse
import json
import os
import re
import sys
from html.parser import HTMLParser
from urllib.parse import unquote, urldefrag

_VOID_TAGS = ('br', 'meta', 'hr', 'img', 'link')
_TYPE_SYNOPSIS = re.compile(r'(class|struct|union|enum|using|typedef|concept)\b')
_IDENTIFIER = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
_SPECIAL_PAGES = ('2constructor', '2destructor', '2conversion')


def StripOverloadSuffix(name):
  """`Sin-0a` -> `Sin`: MrDocs suffixes the pages of overloaded names."""
  return re.sub(r'-0[0-9a-f]+$', '', name)


class Page(HTMLParser):
  """Collects the links, the unlinked texts and the ids of one page."""

  def __init__(self):
    super().__init__(convert_charrefs=True)
    self.ids = set()
    self.links = []  # (href, text, context, section)
    self.texts = []  # (text, context, section), outside of links
    self.code_spans = []  # text of inline code spans
    self._stack = []  # open tags as (name, class)
    self._in_body = False
    self._link = None
    self._code = None

  def _Section(self):
    for tag, css_class in reversed(self._stack):
      if tag == 'div' and css_class:
        return css_class
    return ''

  def _Context(self):
    names = [tag for tag, _ in self._stack]
    if 'h1' in names:
      return 'title'
    if 'pre' in names:
      return 'code-block'
    if 'table' in names:
      return 'table'
    if 'h2' in names or 'h3' in names:
      return 'heading'
    return 'prose'

  def handle_starttag(self, tag, attrs):
    attrs = dict(attrs)
    if 'id' in attrs:
      self.ids.add(attrs['id'])
    if tag == 'body':
      self._in_body = True
    if tag in _VOID_TAGS:
      return
    self._stack.append((tag, attrs.get('class') or ''))
    if tag == 'a':
      is_reference = ('href' in attrs and
                      'mrdocs-anchor' not in (attrs.get('class') or ''))
      self._link = [attrs['href'] if is_reference else None, '',
                    self._Context(), self._Section()]
    elif tag == 'code' and self._Context() != 'code-block':
      self._code = ''

  def handle_endtag(self, tag):
    if tag == 'a' and self._link is not None:
      if self._link[0] is not None:
        self.links.append(tuple(self._link))
      self._link = None
    if tag == 'code' and self._code is not None:
      self.code_spans.append(self._code)
      self._code = None
    for i in range(len(self._stack) - 1, -1, -1):
      if self._stack[i][0] == tag:
        del self._stack[i:]
        break

  def handle_data(self, data):
    names = [tag for tag, _ in self._stack]
    if (not self._in_body or 'script' in names or 'style' in names or
        'footer' in names):
      return
    if self._code is not None:
      self._code += data
    if self._link is not None:
      self._link[1] += data
    else:
      self.texts.append((data, self._Context(), self._Section()))


def ParsePages(root):
  pages = {}
  for directory, _, files in os.walk(root):
    for name in files:
      if name.endswith('.html'):
        path = os.path.join(directory, name)
        page = Page()
        with open(path, encoding='utf-8') as f:
          page.feed(f.read())
        pages[path] = page
  return pages


def FindTypes(root, pages):
  """Returns {simple name: [pages]} of the classes, enums and aliases."""
  types = {}
  for path, page in pages.items():
    synopsis = ' '.join(text for text, context, section in page.texts
                        if context == 'code-block' and section == 'synopsis')
    synopsis = re.sub(r'template\s*<.*?>\s*(requires [^\n]*)?', '', synopsis,
                      flags=re.S).strip()
    if _TYPE_SYNOPSIS.match(synopsis):
      name = StripOverloadSuffix(os.path.basename(path)[:-5])
      types.setdefault(name, []).append(os.path.relpath(path, root))
  return types


def CheckLinks(root, path, page, pages, errors):
  relative = os.path.relpath(path, root)
  for href, text, context, section in page.links:
    if re.match(r'[a-z]+://', href):
      continue
    url, fragment = urldefrag(href)
    target = path
    if url:
      target = os.path.normpath(
          os.path.join(os.path.dirname(path), unquote(url)))
    if target not in pages:
      errors.append(f'{relative}: dead link "{text}" -> {href}')
      continue
    if fragment and fragment not in pages[target].ids:
      errors.append(f'{relative}: dead anchor "{text}" -> {href}')
    if fragment or text.startswith('»') or text.startswith('operator'):
      continue
    target_name = StripOverloadSuffix(os.path.basename(target)[:-5])
    owner = os.path.basename(os.path.dirname(target))
    in_listing = context in ('title', 'table') or section == 'synopsis'
    if target_name in _SPECIAL_PAGES:
      if not in_listing:
        errors.append(f'{relative}: "{text}" links to a constructor or '
                      f'destructor page ({href})')
      elif text.lstrip('~') != owner:
        errors.append(f'{relative}: "{text}" -> {href} (mismatch)')
      continue
    names = _IDENTIFIER.findall(re.sub(r'<.*', '', text.split('(')[0]))
    # An enumerator (`BodyType::kStatic`) links to the page of its enum.
    if target_name not in names[-2:]:
      errors.append(f'{relative}: "{text}" -> {href} (mismatch)')


def CheckTexts(root, path, page, types, errors, review):
  relative = os.path.relpath(path, root)
  own_name = StripOverloadSuffix(os.path.basename(path)[:-5])
  owner = os.path.basename(os.path.dirname(path))
  for span in page.code_spans:
    if re.search(r'\w[.,;:]$', span):
      errors.append(f'{relative}: code span ends with punctuation: "{span}"')
  seen = set()
  for text, context, section in page.texts:
    if context in ('title', 'heading'):
      continue
    if 'internal::' in text:
      errors.append(f'{relative}: names an internal symbol: "{text.strip()}"')
    if context == 'code-block' and section != 'synopsis':
      continue  # @code example
    for match in _IDENTIFIER.finditer(text):
      name = match.group(0)
      if name not in types or (name, context) in seen:
        continue
      if name == own_name and (context == 'code-block' or
                               types[name] == [relative]):
        continue  # the page naming itself
      if (context == 'code-block' and own_name in _SPECIAL_PAGES and
          name == owner):
        continue  # the name of a constructor in its synopsis
      seen.add((name, context))
      start, end = max(0, match.start() - 40), match.end() + 40
      where = 'synopsis' if context == 'code-block' else section
      snippet = ' '.join(text[start:end].split())
      review.append(f'{relative}: {name} ({where}): "{snippet}"')


def main():
  parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
  parser.add_argument('docs_dir', help='output directory of the docs target')
  parser.add_argument('--json', help='also write the report to this file')
  args = parser.parse_args()

  root = os.path.abspath(args.docs_dir)
  pages = ParsePages(root)
  if not pages:
    sys.exit(f'no HTML page in {root}: build the docs target first')
  types = FindTypes(root, pages)
  errors, review = [], []
  for path in sorted(pages):
    CheckLinks(root, path, pages[path], pages, errors)
    CheckTexts(root, path, pages[path], types, errors, review)

  if args.json:
    with open(args.json, 'w', encoding='utf-8') as f:
      json.dump({'errors': errors, 'review': review}, f, indent=1)
  for line in review:
    print('review:', line)
  for line in errors:
    print('error:', line)
  print(f'{len(pages)} pages, {len(types)} types, {len(errors)} errors, '
        f'{len(review)} unlinked type names to review')
  sys.exit(1 if errors else 0)


if __name__ == '__main__':
  main()
