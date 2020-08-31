# flake8: noqa
with section("format"):
    tab_size = 4
    # min_prefix is affected by tab size
    min_prefix_chars = tab_size * 4
    dangle_parens = True  # Dangle Parens to make diffs cleaner
    max_pargs_hwrap = 3  # More aggressive wrapping than defaults


with section("markup"):
    enable_markup = False  # Don't format comments
