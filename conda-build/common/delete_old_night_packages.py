import subprocess
import re
import argparse

MAX_NUMBER_OF_PACKAGES = 30

parser = argparse.ArgumentParser("Running benchmarks on simulator")
parser.add_argument("--username", type=str)
parser.add_argument("--password", type=str)
parser.add_argument('--nightly', help="Remove old conda nightly builds.", action='store_true')
args = parser.parse_args()


if not args.nightly:
    sys.exit()

login_result = subprocess.run(['anaconda', 'login', '--username', args.username, '--password', args.password], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
result = subprocess.run(['anaconda', 'show', 'aihabitat-nightly/habitat-sim'], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
versions = re.findall("\d\.\d\.\d\.\d{4}\.\d\d\.\d\d", str(result.stdout) + str(result.stderr))
remove_versions = versions[:-MAX_NUMBER_OF_PACKAGES]
print(f"anaconda remove {' '.join(list(map(lambda x: f'aihabitat-nightly/habitat-sim/{x}', remove_versions)))}”)
result_remove = subprocess.run(['anaconda', 'remove', "-f", * list(map(lambda x: f'aihabitat-nightly/habitat-sim/{x}', remove_versions))], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
