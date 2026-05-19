# -*- coding: utf-8 -*-
import os
import tempfile


def unique_sibling_path(src_path, suffix="_mdal.tin"):
    base_dir = os.path.dirname(src_path)
    base_name = os.path.splitext(os.path.basename(src_path))[0]
    candidate = os.path.join(base_dir, base_name + suffix)
    if not os.path.exists(candidate):
        return candidate
    n = 1
    while True:
        cand = os.path.join(base_dir, f"{base_name}{suffix[:-4]}_{n}.tin")
        if not os.path.exists(cand):
            return cand
        n += 1


def writable_dir(path):
    try:
        testfile = tempfile.TemporaryFile(dir=path)
        testfile.close()
        return True
    except Exception:
        return False


def default_2dm_path(src):
    base = os.path.splitext(os.path.basename(src))[0]
    candidate = os.path.join(os.path.dirname(src), base + "_editable.2dm")
    if not os.path.exists(candidate):
        return candidate
    n = 1
    while True:
        cand = os.path.join(os.path.dirname(src), base + "_editable_{}.2dm".format(n))
        if not os.path.exists(cand):
            return cand
        n += 1



def unique_path(path):
    if not os.path.exists(path):
        return path
    folder = os.path.dirname(path)
    base, ext = os.path.splitext(os.path.basename(path))
    n = 1
    while True:
        cand = os.path.join(folder, "{}_{}{}".format(base, n, ext))
        if not os.path.exists(cand):
            return cand
        n += 1
