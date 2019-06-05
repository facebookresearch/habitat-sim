import os

import glog

_log_level_mapping = {
    k: v for k, v in zip(range(4), [glog.INFO, glog.WARNING, glog.ERROR, glog.FATAL])
}

if "GLOG_minloglevel" in os.environ:
    glog.setLevel(_log_level_mapping[int(os.environ["GLOG_minloglevel"])])
