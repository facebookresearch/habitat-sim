# Habitat-Matterport 3D Research Dataset


## Downloading with the download script

After getting access to the dataset, you will need to generate a matterport API Token to programatically download the dataset

1. Navigate to https://my.matterport.com/settings/account/devtools
1. Generate an API token
1. Your API token ID then functions as your username, passed to the download script with `--username`,
 and your API token secret functions as your password,
 passed to the download script with `--password`.
 Note: Make sure to write your API token secret down, you can't reveal it again!

To download the minival split, for example, use
```
python -m habitat_sim.utils.datasets_download --username <api-token-id> --password <api-token-secret> --uids hm3d_minival
```


By default the download script will only download what is needed for habitat-sim.  You can add `_full` to the uid to download the raw glbs and the obj+mtl's in addition to what is needed for use with habitat-sim
