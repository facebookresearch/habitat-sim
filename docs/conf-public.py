# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Inherit everything from the local config
import os, sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from conf import *

OUTPUT = "../build/docs-public/habitat-sim/"

HTML_HEADER = """<!-- Global site tag (gtag.js) - Google Analytics -->
<script async src="https://www.googletagmanager.com/gtag/js?id=UA-66408458-4"></script>
<script>
 window.dataLayer = window.dataLayer || [];
 function gtag(){dataLayer.push(arguments);}
 gtag('js', new Date());
 gtag('config', 'UA-66408458-4');
</script>
"""

SEARCH_DOWNLOAD_BINARY = "searchdata-v1.bin"
SEARCH_BASE_URL = "https://aihabitat.org/docs/habitat-sim/"
SEARCH_EXTERNAL_URL = "https://google.com/search?q=site:aihabitat.org+{query}"
