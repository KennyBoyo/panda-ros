#!/bin/bash

USER_ID='JunKhai'
USER_PW='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'

origin=$(git remote get-url origin)
origin_with_pass=${origin/"//"/"//${USER_ID}:${USER_PW}@"}
git push ${origin_with_pass}
