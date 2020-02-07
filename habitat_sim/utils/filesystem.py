import os


def search_dir_tree_for_ext(mydir, ext):
    files_with_ext = []
    for root, dirs, files in os.walk(mydir):
        for f in files:
            if f.endswith(ext):
                files_with_ext.append(os.path.join(root, f))

    return files_with_ext
