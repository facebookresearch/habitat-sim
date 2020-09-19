# flake8: noqa
with section("format"):
    line_width = 88  # Give a little more length
    max_pargs_hwrap = 5
    max_subgroups_hwrap = 3  # So set commands dont get wrapped
    min_prefix_chars = 2
    dangle_parens = True  # Dangle Parens to make diffs cleaner
    always_wrap = [
        "add_library",
        "target_link_libraries",
    ]  # Always wrap these ones
    autosort = True  # Try to autosort lists
with section("markup"):
    enable_markup = False  # Don't format comments
