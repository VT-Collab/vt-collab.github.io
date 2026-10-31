# Collab Lab Website

Source for [collab.me.vt.edu](https://collab.me.vt.edu), built with [Jekyll](https://jekyllrb.com/) and published by GitHub Pages from the `master` branch.

## Preview locally

```bash
bundle config set --local path vendor/bundle   # first time only
bundle install                                  # first time only
bundle exec jekyll serve
```

Then open <http://127.0.0.1:4000/>. To update the Jekyll toolchain to what GitHub Pages runs, run `bundle update github-pages`.

## Editing content

Everything shown on the site comes from the YAML files in `_data/`:

| File | Page |
| --- | --- |
| `news.yml` | News list on the home page (newest first, top five shown) |
| `people.yml`, `ranks.yml` | People page (ranks control section order and labels) |
| `publications.yml` | Publications page, grouped by `year` automatically |
| `values.yml` | Values page |
| `photos.yml` | Photos page, newest first |

## Adding images and PDFs

Keep the site fast: GitHub Pages has a 1 GB size limit and a 100 GB/month bandwidth limit.

- Photos: resize to at most 1600 px on the long edge, save as JPEG (quality ~85). Target under 500 KB.
- People portraits: 300 px wide JPEG.
- Publication figures: at most 800 x 400 px, WebP or PNG.
- Animations: use a muted looping MP4 instead of a GIF (`ffmpeg -i in.gif -pix_fmt yuv420p -crf 26 out.mp4`); the Values and Photos templates render `.mp4` paths as video.
- PDFs: compress to under 5 MB where possible.
