#!/usr/bin/env python3

import os
import git

script_dir = os.path.dirname(os.path.abspath(__file__))

# remove ActionID and StationID from CPM CDD
remove_old_types_patch = os.path.join(script_dir, "remove-old-types.patch")

cpm_cdd_folder = os.path.join(script_dir, "../raw/cpm_ts103324/asn/cdd")
cpm_cdd = git.Repo(cpm_cdd_folder)

# check if there are any changes in the repo
if cpm_cdd.is_dirty():
    # revert patch
    try:
        cpm_cdd.git.execute(['git','apply','-R',remove_old_types_patch])
    except git.GitCommandError as e:
        print(f"{cpm_cdd_folder} is dirty and the patch could not be reverted.")
else:
    # apply patch
    cpm_cdd.git.execute(['git','apply',remove_old_types_patch])